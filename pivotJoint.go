package chipmunk

import (
	"github.com/vova616/chipmunk/transform"
	"github.com/vova616/chipmunk/vect"
	"math"
)

type PivotJoint struct {
	BasicConstraint
	Anchor1, Anchor2 vect.Vect

	r1, r2 vect.Vect
	k1, k2 vect.Vect

	jAcc    vect.Vect
	jMaxLen vect.Float
	bias    vect.Vect
}

func NewPivotJointAnchor(a, b *Body, anchor1, anchor2 vect.Vect) *PivotJoint {
	return &PivotJoint{BasicConstraint: NewConstraint(a, b), Anchor1: anchor1, Anchor2: anchor2}
}

func NewPivotJoint(a, b *Body) *PivotJoint {
	return NewPivotJointAnchor(a, b, vect.Vector_Zero, vect.Vector_Zero)
}

func (this *PivotJoint) PreStep(dt vect.Float) {
	a, b := this.BodyA, this.BodyB

	this.r1 = transform.RotateVect(this.Anchor1, transform.Rotation{a.rot.X, a.rot.Y})
	this.r2 = transform.RotateVect(this.Anchor2, transform.Rotation{b.rot.X, b.rot.Y})

	// Calculate mass tensor
	k_tensor(a, b, this.r1, this.r2, &this.k1, &this.k2)

	// compute max impulse
	this.jMaxLen = this.MaxForce * dt

	// calculate bias velocity
	delta := vect.Sub(vect.Add(b.p, this.r2), vect.Add(a.p, this.r1))

	this.bias = vect.Clamp(vect.Mult(delta, -bias_coef(this.ErrorBias, dt)/dt), this.MaxBias)
}

func bias_coef(errorBias, dt vect.Float) vect.Float {
	return vect.Float(1.0 - math.Pow(float64(errorBias), float64(dt)))
}

func (this *PivotJoint) ApplyCachedImpulse(dt_coef vect.Float) {
	a, b := this.BodyA, this.BodyB
	apply_impulses(a, b, this.r1, this.r2, vect.Mult(this.jAcc, dt_coef))
}

func (this *PivotJoint) ApplyImpulse() {
	a, b := this.BodyA, this.BodyB
	r1, r2 := this.r1, this.r2

	// compute relative velocity
	vr := relative_velocity2(a, b, r1, r2)

	// compute normal impulse
	j := mult_k(vect.Sub(this.bias, vr), this.k1, this.k2)
	jOld := this.jAcc
	this.jAcc = vect.Clamp(vect.Add(this.jAcc, j), this.jMaxLen)
	j = vect.Sub(this.jAcc, jOld)
	// apply impulse
	apply_impulses(a, b, this.r1, this.r2, j)
}

func (this *PivotJoint) Impulse() vect.Float {
	return vect.Length(this.jAcc)
}

func mult_k(vr, k1, k2 vect.Vect) vect.Vect {
	return vect.Vect{vect.Dot(vr, k1), vect.Dot(vr, k2)}
}

func k_tensor(a, b *Body, r1, r2 vect.Vect, k1, k2 *vect.Vect) {
	// calculate mass matrix
	// If I wasn't lazy and wrote a proper matrix class, this wouldn't be so gross...
	m_sum := a.m_inv + b.m_inv

	// start with I*m_sum
	k11 := vect.Float(m_sum)
	k12 := vect.Float(0)
	k21 := vect.Float(0)
	k22 := vect.Float(m_sum)

	// add the influence from r1
	a_i_inv := a.i_inv
	r1xsq := r1.X * r1.X * a_i_inv
	r1ysq := r1.Y * r1.Y * a_i_inv
	r1nxy := -r1.X * r1.Y * a_i_inv
	k11 += r1ysq
	k12 += r1nxy
	k21 += r1nxy
	k22 += r1xsq

	// add the influnce from r2
	b_i_inv := b.i_inv
	r2xsq := r2.X * r2.X * b_i_inv
	r2ysq := r2.Y * r2.Y * b_i_inv
	r2nxy := -r2.X * r2.Y * b_i_inv
	k11 += r2ysq
	k12 += r2nxy
	k21 += r2nxy
	k22 += r2xsq

	// invert
	determinant := (k11 * k22) - (k12 * k21)
	if determinant == 0 {
		panic("Unsolvable constraint.")
	}

	det_inv := 1.0 / determinant
	*k1 = vect.Vect{k22 * det_inv, -k12 * det_inv}
	*k2 = vect.Vect{-k21 * det_inv, k11 * det_inv}
}
