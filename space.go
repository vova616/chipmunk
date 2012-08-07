package chipmunk

import (
	. "chipmunk/vect"
	//. "chipmunk/transform"
	"math"
	"time"
	"fmt"
) 

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

	Bodies []*Body
	AllBodies []*Body

	stamp time.Duration

	contactBuffersHead *ContactBufferHeader
	
	staticShapes *SpatialIndex
	activeShapes *SpatialIndex
	
	cachedArbiters map[HashValue]*Arbiter
	Arbiters []*Arbiter
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
	space.Iterations = 10 
 
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

	return 
}

func (space *Space) Step(dt Float) {

	// don't step if the timestep is 0!
	if dt == 0 {
		return
	}
	
	
	
	bodies := space.Bodies
	 
	space.Arbiters = make([]*Arbiter, 0)

	prev_dt := space.curr_dt
	space.curr_dt = dt

	space.stamp++

	

	for _, body := range bodies {
		body.UpdatePosition(dt)
	}

	space.PushFreshContactBuffer()
	for _, body := range space.AllBodies {
		body.UpdateShapes()
	}
	space.activeShapes.ReindexQuery(spaceCollideShapes, space)

	for _,arb := range space.Arbiters {
	
		a := arb.ShapeA.Body
		b := arb.ShapeB.Body
		
		a.arbiter = arb
		b.arbiter = arb
	}
	
	
	slop := space.collisionSlop;
	biasCoef := Float(1.0 - math.Pow(float64(space.collisionBias), float64(dt)));
	for _,arb := range space.Arbiters {
		arb.preStep(Float(1 / dt),slop,biasCoef)
	}
	
	damping := Float(math.Pow(float64(space.damping), float64(dt)));

	for _, body := range bodies {
		if body.IgnoreGravity { 
			body.UpdateVelocity(Vector_Zero, damping, dt);
			continue
		}
		body.UpdateVelocity(space.Gravity, damping, dt);
	}
	
	
	dt_coef := Float(0)
	if prev_dt != 0 {
		dt_coef = dt/prev_dt
	}
	for _,arb := range space.Arbiters {
		arb.applyCachedImpulse(dt_coef)
	}
	
	fmt.Println("STEP")
	for i:=0; i<10; i++ {
		for _,arb := range space.Arbiters {
			arb.applyImpulse()
		}
	}
	

}

func (space *Space) NewContactBuffer() *ContactBufferHeader {
	return &ContactBufferHeader{}
}

func  spaceCollideShapes(a, b interface{}, space interface{}) {
	SpaceCollideShapes(a.(*Shape), b.(*Shape), space.(*Space))
}

func SpaceCollideShapes(a, b *Shape, space *Space) {
	if queryReject(a,b)  {return }
	
	
	if a.ShapeType() > b.ShapeType() {
		a,b = b,a
	}
	
	
	//cpCollisionHandler *handler = cpSpaceLookupHandler(space, a->collision_type, b->collision_type);
	
	sensor := a.IsSensor || b.IsSensor;
	//if(sensor && handler == &cpDefaultCollisionHandler) return;
	if sensor {
		return
	}

	// Narrow-phase collision detection.
	var contacts *[MaxPoints]Contact = new([MaxPoints]Contact)
	
	for i := 0;i<MaxPoints;i++ {
		contacts[i] = Contact{}
	}
	
	numContacts := collide(contacts, a, b);
	if  numContacts <= 0  {
		 return; // Shapes are not colliding.
	}
	space.contactBuffersHead.numContacts += numContacts
	
	// Get an arbiter from space->arbiterSet for the two shapes.
	// This is where the persistant contact magic comes from.
	
	
	arbHashID := hashPair(a.Hash()*20, b.Hash()*10);
	
	arb := CreateArbiter(a,b) 
	arb.update(contacts, numContacts)
	
	
	_, exist := space.cachedArbiters[arbHashID]
	if exist {
		//println("hash id already exists")
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
	arb.stamp = space.stamp;
}

func  queryReject(a, b *Shape) bool {
	return !TestOverlap(a.BB, b.BB) || a.Body == b.Body || (math.IsInf(float64(a.Body.m), 0) && math.IsInf(float64(b.Body.m), 0))
}

func ContactBufferHeaderInit(header *ContactBufferHeader, stamp time.Duration, splice *ContactBufferHeader) *ContactBufferHeader {

	header.stamp = stamp
	if splice == nil {
		header.next = header
	} else {
		header.next = splice.next
	}
	header.numContacts = 0

	return header
}

func (space *Space) PushFreshContactBuffer() {
	stamp := space.stamp

	head := space.contactBuffersHead

	if head == nil {
		// No buffers have been allocated, make one
		space.contactBuffersHead = ContactBufferHeaderInit(space.NewContactBuffer(), stamp, nil)
	} else if int64(stamp-head.next.stamp) > space.collisionPersistence {
		// The tail buffer is available, rotate the ring
		tail := head.next
		space.contactBuffersHead = ContactBufferHeaderInit(tail, stamp, tail)
	} else {
		// Allocate a new buffer and push it into the ring
		buffer := ContactBufferHeaderInit(space.NewContactBuffer(), stamp, head)
		head.next = buffer
		space.contactBuffersHead = buffer
	}
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
	
	for _,shape := range body.Shapes {
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
	//}

	return shape
}
