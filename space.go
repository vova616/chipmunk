package chipmunk

import (
	"github.com/vova616/chipmunk/vect"
	//. "github.com/vova616/chipmunk/transform"
	"errors"
	"fmt"
	//"github.com/davecgh/go-spew/spew"
	"math"
	"time"
)

const ArbiterBufferSize = 1000
const ContactBufferSize = ArbiterBufferSize * MaxPoints

type Space struct {

	/// Number of iterations to use in the impulse solver to solve contacts.
	Iterations int

	/// Gravity to pass to rigid bodies when integrating velocity.
	Gravity vect.Vect

	/// Damping rate expressed as the fraction of velocity bodies retain each second.
	/// A value of 0.9 would mean that each body's velocity will drop 10% per second.
	/// The default value is 1.0, meaning no damping is applied.
	/// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
	damping vect.Float

	/// Speed threshold for a body to be considered idle.
	/// The default value of 0 means to let the space guess a good threshold based on gravity.
	idleSpeedThreshold vect.Float

	/// Time a group of bodies must remain idle in order to fall asleep.
	/// Enabling sleeping also implicitly enables the the contact graph.
	/// The default value of INFINITY disables the sleeping algorithm.
	sleepTimeThreshold vect.Float

	/// Amount of encouraged penetration between colliding shapes.
	/// Used to reduce oscillating contacts and keep the collision cache warm.
	/// Defaults to 0.1. If you have poor simulation quality,
	/// increase this number as much as possible without allowing visible amounts of overlap.
	collisionSlop vect.Float

	/// Determines how fast overlapping shapes are pushed apart.
	/// Expressed as a fraction of the error remaining after each second.
	/// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
	collisionBias vect.Float

	/// Number of frames that contact information should persist.
	/// Defaults to 3. There is probably never a reason to change this value.
	collisionPersistence int64

	/// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
	/// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
	enableContactGraph bool

	curr_dt vect.Float

	Bodies             []*Body
	sleepingComponents []*Body

	stamp time.Duration

	staticShapes *SpatialIndex
	activeShapes *SpatialIndex

	cachedArbiters map[HashPair]*Arbiter
	Arbiters       []*Arbiter

	ArbiterBuffer []*Arbiter
	ContactBuffer [][]*Contact

	ApplyImpulsesTime time.Duration
	ReindexQueryTime  time.Duration
	StepTime          time.Duration
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

	space.Gravity = vect.Vector_Zero

	space.damping = 1

	space.collisionSlop = 0.5
	space.collisionBias = vect.Float(math.Pow(1.0-0.1, 60))
	space.collisionPersistence = 3

	space.Bodies = make([]*Body, 0)
	space.sleepingComponents = make([]*Body, 0)

	space.staticShapes = NewBBTree(nil)
	space.activeShapes = NewBBTree(space.staticShapes)
	space.cachedArbiters = make(map[HashPair]*Arbiter)
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
	/*
		for i := 0; i < 8; i++ {
			go space.MultiThreadTest()
		}
	*/
	return
}

func (space *Space) Destory() {
	space.Bodies = nil
	space.sleepingComponents = nil
	space.staticShapes = nil
	space.activeShapes = nil
	space.cachedArbiters = nil
	space.Arbiters = nil
	space.ArbiterBuffer = nil
	space.ContactBuffer = nil
}

func (space *Space) Step(dt vect.Float) {

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

	for _, body := range bodies {
		body.UpdatePosition(dt)
	}

	for _, body := range bodies {
		body.UpdateShapes()
		//d := body.Shapes[0].GetAsBox()
		//if d != nil {
		//	fmt.Println(d.verts, body.Shapes[0].AABB(), body.Angle(), body.Position())
		//}
	}

	start := time.Now()
	space.activeShapes.ReindexQuery(spaceCollideShapes, space)
	end := time.Now()

	space.ReindexQueryTime = end.Sub(start)

	//axc := space.activeShapes.SpatialIndexClass.(*BBTree)
	//PrintTree(axc.root)

	for h, arb := range space.cachedArbiters {
		ticks := space.stamp - arb.stamp
		deleted := (arb.BodyA.deleted || arb.BodyB.deleted)
		if (ticks >= 1 && arb.state != arbiterStateCached) || deleted {
			arb.state = arbiterStateCached
			if arb.BodyA.CallbackHandler != nil {
				arb.BodyA.CallbackHandler.CollisionExit(arb)
				if arb.BodyA.deleted {
					arb.BodyA.CallbackHandler = nil
				}
			}
			if arb.BodyB.CallbackHandler != nil {
				arb.BodyB.CallbackHandler.CollisionExit(arb)
				if arb.BodyB.deleted {
					arb.BodyB.CallbackHandler = nil
				}
			}
		}
		if ticks > time.Duration(space.collisionPersistence) || deleted {
			delete(space.cachedArbiters, h)
			space.ArbiterBuffer = append(space.ArbiterBuffer, arb)
			c := arb.Contacts
			if c != nil {
				space.ContactBuffer = append(space.ContactBuffer, c)
			}
		}
	}

	slop := space.collisionSlop
	biasCoef := vect.Float(1.0 - math.Pow(float64(space.collisionBias), float64(dt)))
	for _, arb := range space.Arbiters {
		arb.preStep(vect.Float(1/dt), slop, biasCoef)
	}

	damping := vect.Float(math.Pow(float64(space.damping), float64(dt)))

	for _, body := range bodies {
		if body.IgnoreGravity {
			body.UpdateVelocity(vect.Vector_Zero, damping, dt)
			continue
		}
		body.UpdateVelocity(space.Gravity, damping, dt)
	}

	dt_coef := vect.Float(0)
	if prev_dt != 0 {
		dt_coef = dt / prev_dt
	}
	//fmt.Println(len(space.Arbiters))
	for _, arb := range space.Arbiters {
		arb.applyCachedImpulse(dt_coef)
	}

	//fmt.Println("STEP")
	start = time.Now()

	//fmt.Println("Arbiters", len(space.Arbiters), biasCoef, dt)
	//spew.Config.MaxDepth = 3
	//spew.Config.Indent = "\t"
	for i := 0; i < space.Iterations; i++ {
		for _, arb := range space.Arbiters {
			arb.applyImpulse()
			//spew.Dump(arb)
			//spew.Printf("%+v\n", arb)
		}
	}
	//fmt.Println("####")
	//fmt.Println("")

	//MultiThreadGo()
	//for i:=0; i<8; i++ {
	//	<-done
	//}
	end = time.Now()
	space.ApplyImpulsesTime = end.Sub(start)

	for _, arb := range space.Arbiters {
		if arb.ShapeA.Body.CallbackHandler != nil {
			arb.ShapeA.Body.CallbackHandler.CollisionPostSolve(arb)
		}
		if arb.ShapeB.Body.CallbackHandler != nil {
			arb.ShapeB.Body.CallbackHandler.CollisionPostSolve(arb)
		}
	}

	stepEnd := time.Now()
	space.StepTime = stepEnd.Sub(stepStart)
}

var done = make(chan bool, 8)
var start = make(chan bool, 8)

func (space *Space) MultiThreadTest() {
	for {
		<-start
		for i := 0; i < space.Iterations/8; i++ {
			for _, arb := range space.Arbiters {
				if arb.ShapeA.IsSensor || arb.ShapeB.IsSensor {
					continue
				}
				arb.applyImpulse()
			}
		}
		done <- true
	}
}

func MultiThreadGo() {
	for i := 0; i < 8; i++ {
		start <- true
	}
	for i := 0; i < 8; i++ {
		<-done
	}
}

func PrintTree(node *Node) {
	if node != nil {
		fmt.Println("Parent:")
		fmt.Println(node.bb)
		fmt.Println("A:")
		PrintTree(node.A)
		fmt.Println("B:")
		PrintTree(node.B)
	}
}

func (space *Space) Space() *Space {
	return space
}

func (space *Space) ActiveBody(body *Body) error {
	if body.IsRogue() {
		return errors.New("Internal error: Attempting to activate a rouge body.")
	}

	space.Bodies = append(space.Bodies, body)

	for _, shape := range body.Shapes {
		space.staticShapes.Remove(shape)
		space.activeShapes.Insert(shape)
	}
	/*
		for _, arb := range body.Arbiters {
			bodyA := arb.BodyA
			if body == bodyA || bodyA.IsStatic() {

					int numContacts = arb->numContacts;
					cpContact *contacts = arb->contacts;

					// Restore contact values back to the space's contact buffer memory
					arb->contacts = cpContactBufferGetArray(space);
					memcpy(arb->contacts, contacts, numContacts*sizeof(cpContact));
					cpSpacePushContacts(space, numContacts);

					// Reinsert the arbiter into the arbiter cache
					arbHashID := hashPair(arb.BodyA.Hash()*20, arb.BodyB.Hash()*10)
					space.cachedArbiters[arbHashID] = arb

					// Update the arbiter's state
					arb.stamp = space.stamp
					space->arbiters = append(space->arbiters, arb)

					//cpfree(contacts);

			}
		}
	*/

	return nil
}

func (space *Space) ProcessComponents(dt vect.Float) {

	sleep := math.IsInf(float64(space.sleepTimeThreshold), 0)
	bodies := space.Bodies
	_ = bodies
	if sleep {
		dv := space.idleSpeedThreshold
		dvsq := vect.Float(0)
		if dv == 0 {
			dvsq = dv * dv
		} else {
			dvsq = space.Gravity.LengthSqr() * dt * dt
		}

		for _, body := range space.Bodies {
			keThreshold := vect.Float(0)
			if dvsq != 0 {
				keThreshold = body.m * dvsq
			}
			body.node.IdleTime = 0
			if body.KineticEnergy() <= keThreshold {
				body.node.IdleTime += dt
			}
		}
	}

	for _, arb := range space.Arbiters {
		a, b := arb.BodyA, arb.BodyB
		_, _ = a, b
		if sleep {

		}
	}
	/*
		// Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
		cpArray *arbiters = space->arbiters;
		for(int i=0, count=arbiters->num; i<count; i++){
			cpArbiter *arb = (cpArbiter*)arbiters->arr[i];
			cpBody *a = arb->body_a, *b = arb->body_b;

			if(sleep){
				if((cpBodyIsRogue(b) && !cpBodyIsStatic(b)) || cpBodyIsSleeping(a)) cpBodyActivate(a);
				if((cpBodyIsRogue(a) && !cpBodyIsStatic(a)) || cpBodyIsSleeping(b)) cpBodyActivate(b);
			}

			cpBodyPushArbiter(a, arb);
			cpBodyPushArbiter(b, arb);
		}

		if(sleep){
			// Bodies should be held active if connected by a joint to a non-static rouge body.
			cpArray *constraints = space->constraints;
			for(int i=0; i<constraints->num; i++){
				cpConstraint *constraint = (cpConstraint *)constraints->arr[i];
				cpBody *a = constraint->a, *b = constraint->b;

				if(cpBodyIsRogue(b) && !cpBodyIsStatic(b)) cpBodyActivate(a);
				if(cpBodyIsRogue(a) && !cpBodyIsStatic(a)) cpBodyActivate(b);
			}

			// Generate components and deactivate sleeping ones
			for(int i=0; i<bodies->num;){
				cpBody *body = (cpBody*)bodies->arr[i];

				if(ComponentRoot(body) == NULL){
					// Body not in a component yet. Perform a DFS to flood fill mark 
					// the component in the contact graph using this body as the root.
					FloodFillComponent(body, body);

					// Check if the component should be put to sleep.
					if(!ComponentActive(body, space->sleepTimeThreshold)){
						cpArrayPush(space->sleepingComponents, body);
						CP_BODY_FOREACH_COMPONENT(body, other) cpSpaceDeactivateBody(space, other);

						// cpSpaceDeactivateBody() removed the current body from the list.
						// Skip incrementing the index counter.
						continue;
					}
				}

				i++;

				// Only sleeping bodies retain their component node pointers.
				body->node.root = NULL;
				body->node.next = NULL;
			}
		}
	*/
}

func (space *Space) RemoveBody(body *Body) {
	if body == nil {
		return
	}
	body.BodyActivate()
	for i, pbody := range space.Bodies {
		if pbody == body {
			space.Bodies = append(space.Bodies[:i], space.Bodies[i+1:]...)
			break
		}
	}
	for _, shape := range body.Shapes {
		space.removeShape(shape)
	}
	body.space = nil
	body.Shapes = nil
	body.UserData = nil
	body.deleted = true
}

func (space *Space) removeShape(shape *Shape) {
	shape.space = nil
	if shape.Body.IsStatic() {
		space.staticShapes.Remove(shape)
	} else {
		space.activeShapes.Remove(shape)
	}
	shape.Body = nil
	shape.UserData = nil
	shape.ShapeClass = nil
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

	arb.BodyA = arb.ShapeA.Body
	arb.BodyB = arb.ShapeB.Body

	arb.Surface_vr = vect.Vect{}
	arb.stamp = 0
	//arb.nodeA = new(ArbiterEdge)
	//arb.nodeB = new(ArbiterEdge)
	arb.state = arbiterStateFirstColl
	arb.Contacts = nil
	arb.NumContacts = 0
	arb.e = 0
	arb.u = 0

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

	sensor := a.IsSensor || b.IsSensor
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

	arbHashID := newPair(a, b)

	var arb *Arbiter

	arb, exist := space.cachedArbiters[arbHashID]
	if !exist {
		arb = space.CreateArbiter(a, b)
	}

	var oldContacts []*Contact

	if arb.Contacts != nil {
		oldContacts = arb.Contacts
	}
	arb.update(a, b, contacts, numContacts)
	if oldContacts != nil {
		space.ContactBuffer = append(space.ContactBuffer, oldContacts)
	}

	space.cachedArbiters[arbHashID] = arb

	//cpArbiter *arb = (cpArbiter *)cpHashSetInsert(space->cachedArbiters, arbHashID, shape_pair, space, (cpHashSetTransFunc)cpSpaceArbiterSetTrans);
	//cpArbiterUpdate(arb, contacts, numContacts, handler, a, b);

	// Call the begin function first if it's the first step
	if arb.state == arbiterStateFirstColl {
		// && (!b.Body.CallbackHandler.CollisionEnter(arb) || !a.Body.CallbackHandler.CollisionEnter(arb)
		ignore := false
		if b.Body.CallbackHandler != nil {
			ignore = !b.Body.CallbackHandler.CollisionEnter(arb)
		}
		if a.Body.CallbackHandler != nil {
			ignore = ignore || !a.Body.CallbackHandler.CollisionEnter(arb)
		}
		if ignore {
			arb.Ignore() // permanently ignore the collision until separation 
		}
	}

	preSolveResult := false

	// Ignore the arbiter if it has been flagged
	if arb.state != arbiterStateIgnore {
		// Call preSolve
		if arb.ShapeA.Body.CallbackHandler != nil {
			preSolveResult = arb.ShapeA.Body.CallbackHandler.CollisionPreSolve(arb)
		}
		if arb.ShapeB.Body.CallbackHandler != nil {
			preSolveResult = preSolveResult || arb.ShapeB.Body.CallbackHandler.CollisionPreSolve(arb)
		}
	}

	if preSolveResult &&
		// Process, but don't add collisions for sensors.
		!sensor {
		space.Arbiters = append(space.Arbiters, arb)
	} else {
		//cpSpacePopContacts(space, numContacts);

		space.ContactBuffer = append(space.ContactBuffer, arb.Contacts)
		arb.Contacts = nil
		arb.NumContacts = 0

		// Normally arbiters are set as used after calling the post-solve callback.
		// However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
		if arb.state != arbiterStateIgnore {
			arb.state = arbiterStateNormal
		}
	}

	// Time stamp the arbiter so we know it was used recently.

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
	shape.Update()
	if shape.Body.IsStatic() {
		space.staticShapes.Insert(shape)
	} else {
		space.activeShapes.Insert(shape)
	}

	return shape
}
