package chipmunk

import (
	"github.com/vova616/chipmunk/vect"
	//. "github.com/vova616/chipmunk/transform" 
	"math"
)

type ComponentNode struct {
	Root     *Body
	Next     *Body
	IdleTime vect.Float
}

type BodyType uint8
type UpdatePositionFunction func(body *Body, dt vect.Float)
type UpdateVelocityFunction func(body *Body, gravity vect.Vect, damping, dt vect.Float)

const (
	BodyType_Static  = BodyType(0)
	BodyType_Dynamic = BodyType(1)
)

var Inf = vect.Float(math.Inf(1))

type CollisionCallback interface {
	CollisionEnter(arbiter *Arbiter) bool
	CollisionPreSolve(arbiter *Arbiter) bool
	CollisionPostSolve(arbiter *Arbiter)
	CollisionExit(arbiter *Arbiter)
}

type Body struct {
	/// Mass of the body.
	/// Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason.
	m vect.Float

	/// Mass inverse.
	m_inv vect.Float

	/// Moment of inertia of the body.
	/// Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this reason.
	i vect.Float
	/// Moment of inertia inverse.
	i_inv vect.Float

	/// Position of the rigid body's center of gravity.
	p vect.Vect
	/// Velocity of the rigid body's center of gravity.
	v vect.Vect
	/// Force acting on the rigid body's center of gravity.
	f vect.Vect

	//Transform Transform

	/// Rotation of the body around it's center of gravity in radians.
	/// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
	a vect.Float
	/// Angular velocity of the body around it's center of gravity in radians/second.
	w vect.Float
	/// Torque applied to the body around it's center of gravity.
	t vect.Float

	/// Cached unit length vector representing the angle of the body.
	/// Used for fast rotations using cpvrotate().
	rot vect.Vect

	v_bias vect.Vect
	w_bias vect.Float

	/// User definable data pointer.
	/// Generally this points to your the game object class so you can access it
	/// when given a cpBody reference in a callback.
	UserData           interface{}
	CallbackHandler    CollisionCallback
	UpdatePositionFunc UpdatePositionFunction
	UpdateVelocityFunc UpdateVelocityFunction

	/// Maximum velocity allowed when updating the velocity.
	v_limit vect.Float
	/// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
	w_limit vect.Float

	space *Space

	Shapes []*Shape

	node ComponentNode

	hash HashValue

	deleted bool
	Enabled bool

	idleTime vect.Float

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
	body.Enabled = true

	return
}

func NewBody(mass, i vect.Float) (body *Body) {

	body = &Body{}
	body.Shapes = make([]*Shape, 0)
	body.SetMass(mass)
	body.SetMoment(i)
	body.SetAngle(0)
	body.Enabled = true

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

func (body *Body) KineticEnergy() vect.Float {
	vsq := vect.Dot(body.v, body.v)
	wsq := body.w * body.w
	if vsq != 0 {
		vsq = vsq * body.m
	}
	if wsq != 0 {
		wsq = wsq * body.i
	}
	return vsq + wsq
}

func (body *Body) SetMass(mass vect.Float) {
	if mass <= 0 {
		panic("Mass must be positive and non-zero.")
	}

	body.BodyActivate()
	body.m = mass
	body.m_inv = 1 / mass
}

func (body *Body) SetMoment(moment vect.Float) {
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

func (body *Body) SetAngle(angle vect.Float) {
	body.BodyActivate()
	body.setAngle(angle)
}

func (body *Body) AddAngle(angle float32) {
	body.SetAngle(vect.Float(angle) + body.Angle())
}

func (body *Body) Mass() vect.Float {
	return body.m
}

func (body *Body) setAngle(angle vect.Float) {
	body.a = angle
	body.rot = vect.FromAngle(angle)
}

func (body *Body) BodyActivate() {
	//TODO: make it work with sleeping
	if body.IsStatic() {
		return
	}

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

func (body *Body) SetPosition(pos vect.Vect) {
	body.p = pos
}

func (body *Body) AddForce(x, y float32) {
	body.f.X += vect.Float(x)
	body.f.Y += vect.Float(y)
}

func (body *Body) SetForce(x, y float32) {
	body.f.X = vect.Float(x)
	body.f.Y = vect.Float(y)
}

func (body *Body) AddVelocity(x, y float32) {
	body.v.X += vect.Float(x)
	body.v.Y += vect.Float(y)
}

func (body *Body) SetVelocity(x, y float32) {
	body.v.X = vect.Float(x)
	body.v.Y = vect.Float(y)
}

func (body *Body) AddTorque(t float32) {
	body.t += vect.Float(t)
}

func (body *Body) Torque() float32 {
	return float32(body.t)
}

func (body *Body) VBias() vect.Vect {
	return body.v_bias
}

func (body *Body) WBias() float32 {
	return float32(body.w_bias)
}

func (body *Body) SetVBias(v vect.Vect) {
	body.v_bias = v
}

func (body *Body) SetWBias(w float32) {
	body.w_bias = vect.Float(w)
}

func (body *Body) AngularVelocity() float32 {
	return float32(body.w)
}

func (body *Body) SetTorque(t float32) {
	body.t = vect.Float(t)
}

func (body *Body) AddAngularVelocity(w float32) {
	body.w += vect.Float(w)
}

func (body *Body) SetAngularVelocity(w float32) {
	body.w = vect.Float(w)
}

func (body *Body) Velocity() vect.Vect {
	return body.v
}

func (body *Body) Position() vect.Vect {
	return body.p
}

func (body *Body) Angle() vect.Float {
	return body.a
}

func (body *Body) Rot() (rx, ry float32) {
	return float32(body.rot.X), float32(body.rot.Y)
}

func (body *Body) UpdatePosition(dt vect.Float) {
	if body.UpdatePositionFunc != nil {
		body.UpdatePositionFunc(body, dt)
		return
	}
	body.p = vect.Add(body.p, vect.Mult(vect.Add(body.v, body.v_bias), dt))
	body.setAngle(body.a + (body.w+body.w_bias)*dt)

	body.v_bias = vect.Vector_Zero
	body.w_bias = 0.0
}

func (body *Body) UpdateVelocity(gravity vect.Vect, damping, dt vect.Float) {
	if body.UpdateVelocityFunc != nil {
		body.UpdateVelocityFunc(body, gravity, damping, dt)
		return
	}
	body.v = vect.Add(vect.Mult(body.v, damping), vect.Mult(vect.Add(gravity, vect.Mult(body.f, body.m_inv)), dt))

	body.w = (body.w * damping) + (body.t * body.i_inv * dt)

	body.f = vect.Vector_Zero

}
