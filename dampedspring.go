package chipmunk

import (
	"math"

	"github.com/vova616/chipmunk/transform"
	"github.com/vova616/chipmunk/vect"
)

type DampedSpring struct {
	BasicConstraint

	Anchor1, Anchor2 vect.Vect
	RestLength       vect.Float
	Stiffness        vect.Float
	Damping          vect.Float
	SpringForceFunc  func(*DampedSpring, vect.Float) vect.Float

	targetVRN vect.Float
	vCoef     vect.Float

	r1, r2 vect.Vect
	nMass  vect.Float
	n      vect.Vect
}

func defaultSpringForce(spring *DampedSpring, dist vect.Float) vect.Float {
	return (spring.RestLength - dist) * spring.Stiffness
}

func NewDampedSpring(a, b *Body,
	anchor1, anchor2 vect.Vect,
	restLength, stiffness, damping vect.Float) *DampedSpring {
	return &DampedSpring{
		BasicConstraint: NewConstraint(a, b),
		Anchor1:         anchor1,
		Anchor2:         anchor2,
		SpringForceFunc: defaultSpringForce,
		RestLength:      restLength,
		Stiffness:       stiffness,
		Damping:         damping,
	}
}

func (spring *DampedSpring) PreStep(dt vect.Float) {
	a := spring.BodyA
	b := spring.BodyB

	spring.r1 = transform.RotateVect(spring.Anchor1, transform.Rotation{a.rot.X, a.rot.Y})
	spring.r2 = transform.RotateVect(spring.Anchor2, transform.Rotation{a.rot.X, a.rot.Y})

	delta := vect.Sub(vect.Add(b.p, spring.r2), vect.Add(a.p, spring.r1))
	dist := vect.Length(delta)
	if dist == 0 {
		dist = vect.Float(math.Inf(1))
	}
	spring.n = vect.Mult(delta, 1.0/dist)

	k := k_scalar(a, b, spring.r1, spring.r2, spring.n)
	spring.nMass = 1.0 / k

	spring.targetVRN = 0.0
	spring.vCoef = vect.Float(1.0 - math.Exp(float64(-spring.Damping*dt*k)))

	fSpring := spring.SpringForceFunc(spring, dist)
	apply_impulses(a, b, spring.r1, spring.r2, vect.Mult(spring.n, fSpring*dt))
}

func (spring *DampedSpring) ApplyCachedImpulse(_ vect.Float) {}

func (spring *DampedSpring) ApplyImpulse() {
	a := spring.BodyA
	b := spring.BodyB

	n := spring.n
	r1 := spring.r1
	r2 := spring.r2

	vrn := normal_relative_velocity(a, b, r1, r2, n)

	vDamp := (spring.targetVRN - vrn) * spring.vCoef
	spring.targetVRN = vrn + vDamp

	apply_impulses(a, b, spring.r1, spring.r2, vect.Mult(spring.n, vDamp*spring.nMass))
}

func (spring *DampedSpring) Impulse() vect.Float {
	return 0
}
