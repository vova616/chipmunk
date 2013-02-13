package chipmunk

import "github.com/vova616/chipmunk/vect"

const (
	errorBias = 0.00179701029991443 //cpfpow(1.0f - 0.1f, 60.0f)
)

type ConstraintCallback interface {
	CollisionPreSolve(constraint *Constraint)
	CollisionPostSolve(constraint *Constraint)
}

type Constraint interface {
	PreStep(dt vect.Float)
	ApplyCachedImpulse(dt_coef vect.Float)
	ApplyImpluse()
	Impluse() vect.Float
}

type BasicConstraint struct {
	BodyA, BodyB    *Body
	space           *Space
	maxForce        vect.Float
	errorBias       vect.Float
	maxBias         vect.Float
	CallbackHandler ConstraintCallback
	UserData        Data
}

func NewConstraint(a, b *Body) BasicConstraint {
	return BasicConstraint{BodyA: a, BodyB: b, maxForce: Inf, maxBias: Inf, errorBias: errorBias}
}

func (this *BasicConstraint) PreStep(dt vect.Float) {

}

func (this *BasicConstraint) ApplyCachedImpulse(dt_coef vect.Float) {

}

func (this *BasicConstraint) ApplyImpluse() {

}

func (this *BasicConstraint) Impluse() vect.Float {
	return 0
}
