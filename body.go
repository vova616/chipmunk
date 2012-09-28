package chipmunk

import (
	. "github.com/vova616/chipmunk/vect"
	//. "github.com/vova616/chipmunk/transform" 
	"math"
)

type ComponentNode struct {
	Root     *Body
	Next     *Body
	IdleTime Float
}

type BodyType uint8

const (
	BodyType_Static  = BodyType(0)
	BodyType_Dynamic = BodyType(1)
)

var Inf = Float(math.Inf(1))

type CollisionCallback interface {
	CollisionEnter(arbiter *Arbiter) bool
	CollisionPreSolve(arbiter *Arbiter) bool
	CollisionPostSolve(arbiter *Arbiter)
	CollisionExit(arbiter *Arbiter)
}

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

	v_bias Vect
	w_bias Float

	/// User definable data pointer.
	/// Generally this points to your the game object class so you can access it
	/// when given a cpBody reference in a callback.
	UserData        interface{}
	CallbackHandler CollisionCallback

	/// Maximum velocity allowed when updating the velocity.
	v_limit Float
	/// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
	w_limit Float

	space *Space

	Shapes []*Shape

	node ComponentNode

	hash HashValue

	deleted bool

	idleTime Float

	IgnoreGravity bool
}

func NewBodyStatic() (body *Body) {

	body = &Body{}
	body.Shapes = make([]*Shape, 0)
	body.SetMass(Inf)
	body.SetMoment(Inf)
	body.IgnoreGravity = true
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

func (body *Body) Clone() *Body {
	clone := *body
	clone.Shapes = make([]*Shape, 0)
	for _, shape := range body.Shapes {
		clone.AddShape(shape.Clone())
	}
	clone.space = nil
	clone.hash = 0
	return &clone
}

func (body *Body) KineticEnergy() Float {
	vsq := Dot(body.v, body.v)
	wsq := body.w * body.w
	if vsq != 0 {
		vsq = vsq * body.m
	}
	if wsq != 0 {
		wsq = wsq * body.i
	}
	return vsq + wsq
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

func (body *Body) Moment() float32 {
	return float32(body.i)
}

func (body *Body) MomentIsInf() bool {
	return math.IsInf(float64(body.i), 0)
}

func (body *Body) SetAngle(angle Float) {
	body.BodyActivate()
	body.setAngle(angle)
}

func (body *Body) AddAngle(angle float32) {
	body.SetAngle(Float(angle) + body.Angle())
}

func (body *Body) Mass() Float {
	return body.m
}

func (body *Body) setAngle(angle Float) {
	body.a = angle
	body.rot = FromAngle(angle)
}

func (body *Body) BodyActivate() {
	if !body.IsRogue() {
		body.node.IdleTime = 0

	}
}

func (body *Body) ComponentRoot() *Body {
	if body != nil {
		return body.node.Root
	}
	return nil
}

func (body *Body) ComponentActive() {
	if body.IsSleeping() || body.IsRogue() {
		return
	}
	return

	space := body.space
	b := body
	for b != nil {
		next := b.node.Next

		b.node.IdleTime = 0
		b.node.Root = nil
		b.node.Next = nil
		space.ActiveBody(body)

		b = next
	}

	//for i,sleeping
	//cpArrayDeleteObj(space->sleepingComponents, root);
}

func (body *Body) IsRogue() bool {
	return body.space == nil
}

func (body *Body) IsSleeping() bool {
	return body.node.Root != nil
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

func (body *Body) AddForce(x, y float32) {
	body.f.X += Float(x)
	body.f.Y += Float(y)
}

func (body *Body) SetForce(x, y float32) {
	body.f.X = Float(x)
	body.f.Y = Float(y)
}

func (body *Body) AddVelocity(x, y float32) {
	body.v.X += Float(x)
	body.v.Y += Float(y)
}

func (body *Body) SetVelocity(x, y float32) {
	body.v.X = Float(x)
	body.v.Y = Float(y)
}

func (body *Body) AddTorque(t float32) {
	body.t += Float(t)
}

func (body *Body) SetTorque(t float32) {
	body.t = Float(t)
}

func (body *Body) AddAngularVelocity(w float32) {
	body.w += Float(w)
}

func (body *Body) SetAngularVelocity(w float32) {
	body.w = Float(w)
}

func (body *Body) Velocity() Vect {
	return body.v
}

func (body *Body) Position() Vect {
	return body.p
}

func (body *Body) Angle() Float {
	return body.a
}

func (body *Body) Rot() (rx, ry float32) {
	return float32(body.rot.X), float32(body.rot.Y)
}

func (body *Body) UpdatePosition(dt Float) {

	body.p = Add(body.p, Mult(Add(body.v, body.v_bias), dt))
	body.setAngle(body.a + (body.w+body.w_bias)*dt)

	body.v_bias = Vector_Zero
	body.w_bias = 0.0
}

func (body *Body) UpdateVelocity(gravity Vect, damping, dt Float) {

	body.v = Add(Mult(body.v, damping), Mult(Add(gravity, Mult(body.f, body.m_inv)), dt))

	body.w = (body.w * damping) + (body.t * body.i_inv * dt)

	body.f = Vector_Zero

}
