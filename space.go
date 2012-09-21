package chipmunk

import (
	. "github.com/vova616/chipmunk/vect"
	//. "github.com/vova616/chipmunk/transform"
	"fmt"
	"math"
	"time"
)

const Debug = false
const ArbiterBufferSize = 1000
const ContactBufferSize = ArbiterBufferSize * MaxPoints

type Space struct {

	/// Number of iterations to use in the impulse solver to solve contacts.
	Iterations int

	/// Gravity to pass to rigid bodies when integrating velocity.
	Gravity Vect

	/// Damping rate expressed as the fraction of velocity bodies retain each second.
	/// A value of 0.9 would mean that each body's velocity will drop 10% per second.
	/// The default value is 1.0, meaning no damping is applied.
	/// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
	damping Float

	/// Speed threshold for a body to be considered idle.
	/// The default value of 0 means to let the space guess a good threshold based on gravity.
	idleSpeedThreshold Float

	/// Time a group of bodies must remain idle in order to fall asleep.
	/// Enabling sleeping also implicitly enables the the contact graph.
	/// The default value of INFINITY disables the sleeping algorithm.
	sleepTimeThreshold Float

	/// Amount of encouraged penetration between colliding shapes.
	/// Used to reduce oscillating contacts and keep the collision cache warm.
	/// Defaults to 0.1. If you have poor simulation quality,
	/// increase this number as much as possible without allowing visible amounts of overlap.
	collisionSlop Float

	/// Determines how fast overlapping shapes are pushed apart.
	/// Expressed as a fraction of the error remaining after each second.
	/// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
	collisionBias Float

	/// Number of frames that contact information should persist.
	/// Defaults to 3. There is probably never a reason to change this value.
	collisionPersistence int64

	/// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
	/// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
	enableContactGraph bool

	curr_dt Float

	Bodies    []*Body
	AllBodies []*Body

	stamp time.Duration

	staticShapes *SpatialIndex
	activeShapes *SpatialIndex

	cachedArbiters map[HashValue]*Arbiter
	Arbiters       []*Arbiter

	ArbiterBuffer []*Arbiter
	ContactBuffer [][]*Contact
}

type ContactBufferHeader struct {
	stamp       time.Duration
	next        *ContactBufferHeader
	numContacts int
}

type ContactBuffer struct {
	header   ContactBufferHeader
	contacts [256]Contact
}

func NewSpace() (space *Space) {

	space = &Space{}
	space.Iterations = 20

	space.Gravity = Vector_Zero

	space.damping = 1

	space.collisionSlop = 0.5
	space.collisionBias = Float(math.Pow(1.0-0.1, 60))
	space.collisionPersistence = 3

	space.Bodies = make([]*Body, 0)

	space.staticShapes = NewBBTree(nil)
	space.activeShapes = NewBBTree(space.staticShapes)
	space.cachedArbiters = make(map[HashValue]*Arbiter)
	space.Arbiters = make([]*Arbiter, 0)
	space.ArbiterBuffer = make([]*Arbiter, ArbiterBufferSize)

	for i := 0; i < len(space.ArbiterBuffer); i++ {
		space.ArbiterBuffer[i] = newArbiter()
	}

	space.ContactBuffer = make([][]*Contact, ContactBufferSize)

	for i := 0; i < len(space.ContactBuffer); i++ {
		var contacts []*Contact = make([]*Contact, MaxPoints)

		for i := 0; i < MaxPoints; i++ {
			contacts[i] = &Contact{}
		}
		space.ContactBuffer[i] = contacts
	}

	return
}

func (space *Space) Step(dt Float) {

	// don't step if the timestep is 0!
	if dt == 0 {
		return
	}

	stepStart := time.Now()

	bodies := space.Bodies

	for _, arb := range space.Arbiters {
		arb.state = arbiterStateNormal
	}

	space.Arbiters = space.Arbiters[0:0]

	prev_dt := space.curr_dt
	space.curr_dt = dt

	space.stamp++

	for h, arb := range space.cachedArbiters {
		if space.stamp-arb.stamp > time.Duration(space.collisionPersistence) {
			delete(space.cachedArbiters, h)
			space.ArbiterBuffer = append(space.ArbiterBuffer, arb)
			c := arb.Contacts
			arb.Contacts = nil
			arb.NumContacts = 0
			space.ContactBuffer = append(space.ContactBuffer, c)
		}
	}

	for _, body := range bodies {
		body.UpdatePosition(dt)
	}

	for _, body := range space.AllBodies {
		body.UpdateShapes()
	}

	start := time.Now()
	space.activeShapes.ReindexQuery(spaceCollideShapes, space)
	end := time.Now()
	if Debug {
		fmt.Println("spaceCollideShapes time", end.Sub(start))
	}

	for _, arb := range space.Arbiters {

		a := arb.ShapeA.Body
		b := arb.ShapeB.Body

		a.arbiter = arb
		b.arbiter = arb
	}

	slop := space.collisionSlop
	biasCoef := Float(1.0 - math.Pow(float64(space.collisionBias), float64(dt)))
	for _, arb := range space.Arbiters {
		arb.preStep(Float(1/dt), slop, biasCoef)
	}

	damping := Float(math.Pow(float64(space.damping), float64(dt)))

	for _, body := range bodies {
		if body.IgnoreGravity {
			body.UpdateVelocity(Vector_Zero, damping, dt)
			continue
		}
		body.UpdateVelocity(space.Gravity, damping, dt)
	}

	dt_coef := Float(0)
	if prev_dt != 0 {
		dt_coef = dt / prev_dt
	}
	//fmt.Println(len(space.Arbiters))
	for _, arb := range space.Arbiters {
		arb.applyCachedImpulse(dt_coef)
	}

	//fmt.Println("STEP")
	start = time.Now()

	//done := make(chan bool, 8)

	//for j:=0;j<8;j++ {
	//go func() {
	for i := 0; i < space.Iterations; i++ {
		for _, arb := range space.Arbiters {
			if arb.ShapeA.IsSensor || arb.ShapeB.IsSensor {
				continue
			}
			arb.applyImpulse()
		}
	}
	//	done <- true
	//	}()
	//}

	//for i:=0; i<8; i++ {
	//	<-done
	//}

	end = time.Now()
	stepEnd := time.Now()
	if Debug {
		fmt.Println("applyImpulse time", end.Sub(start), "space.Arbiters", len(space.Arbiters))
		fmt.Println("stepTime", stepEnd.Sub(stepStart))
	}

}

func (space *Space) Space() *Space {
	return space
}

// Creates an arbiter between the given shapes.
// If the shapes do not collide, arbiter.NumContact is zero.
func (space *Space) CreateArbiter(sa, sb *Shape) *Arbiter {

	var arb *Arbiter
	if len(space.ArbiterBuffer) > 0 {
		arb, space.ArbiterBuffer = space.ArbiterBuffer[len(space.ArbiterBuffer)-1], space.ArbiterBuffer[:len(space.ArbiterBuffer)-1]
	} else {
		for i := 0; i < ArbiterBufferSize/2; i++ {
			space.ArbiterBuffer = append(space.ArbiterBuffer, newArbiter())
		}
		arb = newArbiter()
	}
	//arb = newArbiter()

	if sa.ShapeType() > sb.ShapeType() {
		arb.ShapeA = sb
		arb.ShapeB = sa
	} else {
		arb.ShapeA = sa
		arb.ShapeB = sb
	}

	arb.Surface_vr = Vect{}
	arb.stamp = 0
	arb.nodeA = new(ArbiterEdge)
	arb.nodeB = new(ArbiterEdge)
	arb.state = arbiterStateFirstColl

	return arb
}

func spaceCollideShapes(a, b Indexable, null Data) {
	SpaceCollideShapes(a.Shape(), b.Shape(), a.Shape().space)
}

func SpaceCollideShapes(a, b *Shape, space *Space) {
	if queryReject(a, b) {
		return
	}

	if a.ShapeType() > b.ShapeType() {
		a, b = b, a
	}

	//cpCollisionHandler *handler = cpSpaceLookupHandler(space, a->collision_type, b->collision_type);

	//sensor := a.IsSensor || b.IsSensor;
	//if(sensor && handler == &cpDefaultCollisionHandler) return;
	//if sensor {
	//	return
	//}

	// Narrow-phase collision detection.

	var contacts []*Contact

	if len(space.ContactBuffer) > 0 {
		contacts, space.ContactBuffer = space.ContactBuffer[len(space.ContactBuffer)-1], space.ContactBuffer[:len(space.ContactBuffer)-1]
	} else {
		for i := 0; i < ContactBufferSize/2; i++ {
			ccs := make([]*Contact, MaxPoints)

			for i := 0; i < MaxPoints; i++ {
				ccs[i] = &Contact{}
			}
			space.ContactBuffer = append(space.ContactBuffer, ccs)
		}
		contacts, space.ContactBuffer = space.ContactBuffer[len(space.ContactBuffer)-1], space.ContactBuffer[:len(space.ContactBuffer)-1]
	}

	numContacts := collide(contacts, a, b)
	if numContacts <= 0 {
		space.ContactBuffer = append(space.ContactBuffer, contacts)
		return // Shapes are not colliding.
	}

	contacts = contacts[:numContacts]

	// Get an arbiter from space->arbiterSet for the two shapes.
	// This is where the persistant contact magic comes from.

	arbHashID := hashPair(a.Hash()*20, b.Hash()*10)

	var arb *Arbiter

	arbt, exist := space.cachedArbiters[arbHashID]
	if exist &&
		((arbt.ShapeA == a && arbt.ShapeB == b) ||
			(arbt.ShapeA == b && arbt.ShapeB == a)) {
		arb = arbt
	} else {
		arb = space.CreateArbiter(a, b)
	}

	var oldContacts []*Contact

	if arb.Contacts != nil {
		oldContacts = arb.Contacts
	}
	arb.update(contacts, numContacts)
	if oldContacts != nil {
		space.ContactBuffer = append(space.ContactBuffer, oldContacts)
	}

	space.cachedArbiters[arbHashID] = arb

	space.Arbiters = append(space.Arbiters, arb)

	//cpArbiter *arb = (cpArbiter *)cpHashSetInsert(space->cachedArbiters, arbHashID, shape_pair, space, (cpHashSetTransFunc)cpSpaceArbiterSetTrans);
	//cpArbiterUpdate(arb, contacts, numContacts, handler, a, b);

	// Call the begin function first if it's the first step
	//if(arb->state == cpArbiterStateFirstColl && !handler->begin(arb, space, handler->data)){
	//	cpArbiterIgnore(arb); // permanently ignore the collision until separation
	//}
	/*
		if(
			// Ignore the arbiter if it has been flagged
			(arb->state != cpArbiterStateIgnore) && 
			// Call preSolve
			handler->preSolve(arb, space, handler->data) &&
			// Process, but don't add collisions for sensors.
			!sensor
		){

			cpArrayPush(space->arbiters, arb);
		} else {
			cpSpacePopContacts(space, numContacts);

			arb->contacts = NULL;
			arb->numContacts = 0;

			// Normally arbiters are set as used after calling the post-solve callback.
			// However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
			if(arb->state != cpArbiterStateIgnore) arb->state = cpArbiterStateNormal;
		}

		// Time stamp the arbiter so we know it was used recently.

	*/
	arb.stamp = space.stamp
}

func queryReject(a, b *Shape) bool {
	//|| (a.Layer & b.Layer) != 0
	return a.Body == b.Body || (a.Group != 0 && a.Group == b.Group) || (a.Layer&b.Layer) == 0 || (math.IsInf(float64(a.Body.m), 0) && math.IsInf(float64(b.Body.m), 0)) || !TestOverlap(a.BB, b.BB)
}

func (space *Space) AddBody(body *Body) *Body {
	if body.space != nil {
		println("This body is already added to a space and cannot be added to another.")
		return body
	}

	body.space = space
	if !body.IsStatic() {
		space.Bodies = append(space.Bodies, body)
	}
	space.AllBodies = append(space.AllBodies, body)

	for _, shape := range body.Shapes {
		if shape.space == nil {
			space.AddShape(shape)
		}
	}

	return body
}

func (space *Space) AddShape(shape *Shape) *Shape {
	if shape.space != nil {
		println("This shape is already added to a space and cannot be added to another.")
		return shape
	}

	shape.space = space
	//shape.Update()
	//if shape.Body.IsStatic() {
	//	space.staticShapes.Insert(shape)
	//} else {
	space.activeShapes.Insert(shape)
	///}

	return shape
}
