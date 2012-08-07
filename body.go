package chipmunk

import (
	. "chipmunk/vect"
	//. "chipmunk/transform"
	"math"
)

type ComponentNode struct {
	Root     *Body
	Next     *Body
	IdleTime Float
}

type BodyType uint8 

const (
	BodyType_Static = BodyType(0)
	BodyType_Dynamic = BodyType(1)
	
)

var Inf = Float(math.Inf(1))

type Body struct {
	/// Mass of the body.
	/// Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason.
	m Float

	/// Mass inverse.
	m_inv Float

	/// Moment of inertia of the body.
	/// Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this reason.
	i Float
	/// Moment of inertia inverse.
	i_inv Float

	/// Position of the rigid body's center of gravity.
	p Vect
	/// Velocity of the rigid body's center of gravity.
	v Vect
	/// Force acting on the rigid body's center of gravity.
	f Vect
	
	//Transform Transform

	/// Rotation of the body around it's center of gravity in radians.
	/// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
	a Float
	/// Angular velocity of the body around it's center of gravity in radians/second.
	w Float
	/// Torque applied to the body around it's center of gravity.
	t Float

	/// Cached unit length vector representing the angle of the body.
	/// Used for fast rotations using cpvrotate().
	rot Vect

	/// User definable data pointer.
	/// Generally this points to your the game object class so you can access it
	/// when given a cpBody reference in a callback.
	UserData interface{}

	/// Maximum velocity allowed when updating the velocity.
	v_limit Float
	/// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
	w_limit Float

	v_bias Vect
	w_bias Float

	space *Space
	
	Shapes []*Shape

	node ComponentNode
	
	hash HashValue
	
	arbiter *Arbiter
	
	idleTime Float
	
	IgnoreGravity bool
}

func NewBodyStatic() (body *Body) {

	body = &Body{}
	body.Shapes = make([]*Shape, 0)
	body.SetMass(Inf) 
	body.SetMoment(Inf)
	body.node.IdleTime = Inf
	body.SetAngle(0)

	return
}

func NewBody(mass, i Float) (body *Body) {

	body = &Body{}
	body.Shapes = make([]*Shape, 0)
	body.SetMass(mass) 
	body.SetMoment(i)
	body.SetAngle(0)
	return
}

func (body *Body) AddShape(shape *Shape) {
	body.Shapes = append(body.Shapes, shape)
	shape.Body = body
}

func (body *Body) SetMass(mass Float) {
	if mass <= 0 {
		panic("Mass must be positive and non-zero.")
	}

	body.BodyActivate()
	body.m = mass
	body.m_inv = 1 / mass
}

func (body *Body) SetMoment(moment Float) {
	if moment <= 0 {
		panic("Moment of Inertia must be positive and non-zero.")
	}

	body.BodyActivate()
	body.i = moment
	body.i_inv = 1 / moment
}

func (body *Body) SetAngle(angle Float) {
	body.BodyActivate()
	body.setAngle(angle)
}

func (body *Body) setAngle(angle Float) {
	body.a = angle
	body.rot = FromAngle(angle)
}

func (body *Body) BodyActivate() {
	if body.IsStatic() {
		return
	}
	body.node.IdleTime = 0
 
}

func (body *Body) IsStatic() bool {
	return math.IsInf(float64(body.node.IdleTime), 0)
}


func (body *Body) UpdateShapes() {
	for _, shape := range body.Shapes {
		shape.Update()
	}
}

func (body *Body) SetPosition(pos Vect) {
	body.p = pos
}

func (body *Body) AddForce(force Vect) {
	body.f.Add(force)
}

func (body *Body) SetForce(force Vect) {
	body.f = force
}


func (body *Body) Position() Vect{
	return body.p
}

func (body *Body) Angle() Float {
	return body.a
}

func (body *Body) UpdatePosition(dt Float) {

	body.p = Add(body.p, Mult(Add(body.v, body.v_bias), dt))
	body.setAngle(body.a + (body.w+body.w_bias)*dt)

	body.v_bias = Vector_Zero
	body.w_bias = 0.0
}

func (body *Body) UpdateVelocity(gravity Vect, damping, dt Float) {
	
	body.v = Add(Mult(body.v, damping), Mult(Add(gravity, Mult(body.f, body.m_inv)), dt))
	
	body.w = body.w*damping + body.t*body.i_inv*dt

}


