package chipmunk

import (
	"github.com/vova616/chipmunk/vect"
)

/*
SimpleMotor represents a joint that will rotate an object relative to another while also correctly moving it forward.
Most useful for turning wheels.
*/
type SimpleMotor struct {
    BasicConstraint
    iSum vect.Float
    jAcc vect.Float
    rate vect.Float
}

/*
PreStep calculates necessary values before the physics step is applied.
*/
func (joint *SimpleMotor) PreStep(dt vect.Float) {
	a := joint.BodyA
	b := joint.BodyB
	
	// calculate moment of inertia coefficient.
	joint.iSum = 1.0/(a.i_inv + b.i_inv)
}

/*
ApplyCachedImpulse applies the cached impulse to the connected bodies.
*/
func (joint *SimpleMotor) ApplyCachedImpulse(dt_coef vect.Float) {
	a := joint.BodyA
	b := joint.BodyB
	
	j := joint.jAcc*dt_coef
	a.w -= j*a.i_inv
	b.w += j*b.i_inv
}

/*
ApplyImpule applies the motors impulse to the connected bodies.
*/
func (joint *SimpleMotor) ApplyImpulse() {
	a := joint.BodyA
	b := joint.BodyB
	
	// compute relative rotational velocity
	wr := b.w - a.w + joint.rate
	
	jMax := joint.MaxForce
	
	// compute normal impulse
	j := -wr*joint.iSum
	jOld := joint.jAcc
	joint.jAcc = clamp(jOld + j, -jMax, jMax)
	j = joint.jAcc - jOld
	
	// apply impulse
	a.w -= j*a.i_inv
	b.w += j*b.i_inv
}

func clamp(value, min, max vect.Float) vect.Float {
    if value > max {
        return max
    }
    
    if value < min {
        return min
    }
    
    return value
}

/*
Impulse returns the motors impulse.
*/
func (joint *SimpleMotor) Impulse() vect.Float {
	if joint.jAcc < 0 {
        return -joint.jAcc
    }
    
    return joint.jAcc
}

/*
GetRate returns the motor rate.
*/
func (joint *SimpleMotor) GetRate() vect.Float {
    return joint.rate
}

/*
SetRate sets the motor rate (aka power).
*/
func (joint *SimpleMotor) SetRate(rate vect.Float) {
    joint.rate = rate
}

/*
NewSimpleMotor creates a new motor joint.
*/
func NewSimpleMotor(a, b *Body, rate vect.Float) *SimpleMotor {
    return &SimpleMotor{BasicConstraint: NewConstraint(a, b), rate: rate, jAcc: 0.0}
}