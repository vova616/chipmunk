package chipmunk

import (
	"github.com/vova616/chipmunk/vect"
	"log"
	"math"
	//"fmt"
)

type collisionHandler func(contacts []*Contact, sA, sB *Shape) int

var collisionHandlers = [numShapes][numShapes]collisionHandler{
	ShapeType_Circle: [numShapes]collisionHandler{
		ShapeType_Circle:  circle2circle,
		ShapeType_Segment: circle2segment,
		ShapeType_Polygon: circle2polygon,
		ShapeType_Box:     circle2box,
	},
	ShapeType_Segment: [numShapes]collisionHandler{
		ShapeType_Circle:  nil,
		ShapeType_Segment: nil,
		ShapeType_Polygon: segment2polygon,
		ShapeType_Box:     segment2box,
	},
	ShapeType_Polygon: [numShapes]collisionHandler{
		ShapeType_Circle:  nil,
		ShapeType_Segment: nil,
		ShapeType_Polygon: polygon2polygon,
		ShapeType_Box:     polygon2box,
	},
	ShapeType_Box: [numShapes]collisionHandler{
		ShapeType_Circle:  nil,
		ShapeType_Segment: nil,
		ShapeType_Polygon: nil,
		ShapeType_Box:     box2box,
	},
}

func collide(contacts []*Contact, sA, sB *Shape) int {
	contacts = contacts[:MaxPoints]
	stA := sA.ShapeType()
	stB := sB.ShapeType()

	if stA > stB {
		log.Printf("sta: %v, stb: %v", stA, stB)
		log.Printf("Error: shapes not ordered")
		return 0
	}

	handler := collisionHandlers[stA][stB]
	if handler == nil {
		return 0
	}

	return handler(contacts, sA, sB)
}

//START COLLISION HANDLERS
func circle2circle(contacts []*Contact, sA, sB *Shape) int {
	csA, ok := sA.ShapeClass.(*CircleShape)
	if !ok {
		log.Printf("Error: ShapeA not a CircleShape!")
		return 0
	}
	csB, ok := sB.ShapeClass.(*CircleShape)
	if !ok {
		log.Printf("Error: ShapeA not a CircleShape!")
		return 0
	}
	return circle2circleQuery(csA.Tc, csB.Tc, csA.Radius, csB.Radius, contacts[0])
}

func circle2segment(contacts []*Contact, sA, sB *Shape) int {
	circle, ok := sA.ShapeClass.(*CircleShape)
	if !ok {
		log.Printf("Error: ShapeA not a CircleShape!")
		return 0
	}
	segment, ok := sB.ShapeClass.(*SegmentShape)
	if !ok {
		log.Printf("Error: ShapeB not a SegmentShape!")
		return 0
	}

	return circle2segmentFunc(contacts, circle, segment)
}

func circle2polygon(contacts []*Contact, sA, sB *Shape) int {
	circle, ok := sA.ShapeClass.(*CircleShape)
	if !ok {
		log.Printf("Error: ShapeA not a CircleShape!")
		return 0
	}
	poly, ok := sB.ShapeClass.(*PolygonShape)
	if !ok {
		log.Printf("Error: ShapeB not a PolygonShape!")
		return 0
	}

	return circle2polyFunc(contacts, circle, poly)
}

func segment2polygon(contacts []*Contact, sA, sB *Shape) int {
	segment, ok := sA.ShapeClass.(*SegmentShape)
	if !ok {
		log.Printf("Error: ShapeA not a SegmentShape!")
		return 0
	}
	poly, ok := sB.ShapeClass.(*PolygonShape)
	if !ok {
		log.Printf("Error: ShapeB not a PolygonShape!")
		return 0
	}
	return seg2polyFunc(contacts, segment, poly)
}

func polygon2polygon(contacts []*Contact, sA, sB *Shape) int {
	poly1, ok := sA.ShapeClass.(*PolygonShape)
	if !ok {
		log.Printf("Error: ShapeA not a PolygonShape!")
		return 0
	}
	poly2, ok := sB.ShapeClass.(*PolygonShape)
	if !ok {
		log.Printf("Error: ShapeB not a PolygonShape!")
		return 0
	}

	return poly2polyFunc(contacts, poly1, poly2)
}

func circle2box(contacts []*Contact, sA, sB *Shape) int {
	circle, ok := sA.ShapeClass.(*CircleShape)
	if !ok {
		log.Printf("Error: ShapeA not a CircleShape!")
		return 0
	}
	box, ok := sB.ShapeClass.(*BoxShape)
	if !ok {
		log.Printf("Error: ShapeB not a BoxShape!")
		return 0
	}

	return circle2polyFunc(contacts, circle, box.Polygon)
}

func segment2box(contacts []*Contact, sA, sB *Shape) int {
	seg, ok := sA.ShapeClass.(*SegmentShape)
	if !ok {
		log.Printf("Error: ShapeA not a SegmentShape!")
		return 0
	}
	box, ok := sB.ShapeClass.(*BoxShape)
	if !ok {
		log.Printf("Error: ShapeB not a BoxShape!")
		return 0
	}

	return seg2polyFunc(contacts, seg, box.Polygon)
}

func polygon2box(contacts []*Contact, sA, sB *Shape) int {
	poly, ok := sA.ShapeClass.(*PolygonShape)
	if !ok {
		log.Printf("Error: ShapeA not a PolygonShape!")
		return 0
	}
	box, ok := sB.ShapeClass.(*BoxShape)
	if !ok {
		log.Printf("Error: ShapeB not a BoxShape!")
		return 0
	}

	return poly2polyFunc(contacts, poly, box.Polygon)
}

func box2box(contacts []*Contact, sA, sB *Shape) int {
	box1, ok := sA.ShapeClass.(*BoxShape)
	if !ok {
		log.Printf("Error: ShapeA not a BoxShape!")
		return 0
	}
	box2, ok := sB.ShapeClass.(*BoxShape)
	if !ok {
		log.Printf("Error: ShapeB not a BoxShape!")
		return 0
	}

	return poly2polyFunc(contacts, box1.Polygon, box2.Polygon)
}

//END COLLISION HANDLERS

func circle2circleQuery(p1, p2 vect.Vect, r1, r2 vect.Float, con *Contact) int {
	minDist := r1 + r2

	delta := vect.Sub(p2, p1)
	distSqr := delta.LengthSqr()

	if distSqr >= minDist*minDist {
		return 0
	}

	dist := vect.Float(math.Sqrt(float64(distSqr)))

	pDist := dist
	if dist == 0.0 {
		pDist = vect.Float(math.Inf(1))
	}

	pos := vect.Add(p1, vect.Mult(delta, 0.5+(r1-0.5*minDist)/pDist))

	norm := vect.Vect{1, 0}

	if dist != 0.0 {
		norm = vect.Mult(delta, 1.0/dist)
	}

	con.reset(pos, norm, dist-minDist, 0)

	return 1
}

func segmentEncapQuery(p1, p2 vect.Vect, r1, r2 vect.Float, con *Contact, tangent vect.Vect) int {
	count := circle2circleQuery(p1, p2, r1, r2, con)
	if vect.Dot(con.n, tangent) >= 0.0 {
		return count
	} else {
		return 0
	}
	panic("Never reached")
}

func circle2segmentFunc(contacts []*Contact, circle *CircleShape, segment *SegmentShape) int {
	rsum := circle.Radius + segment.Radius

	//Calculate normal distance from segment
	dn := vect.Dot(segment.Tn, circle.Tc) - vect.Dot(segment.Ta, segment.Tn)
	dist := vect.FAbs(dn) - rsum
	if dist > 0.0 {
		return 0
	}

	//Calculate tangential distance along segment
	dt := -vect.Cross(segment.Tn, circle.Tc)
	dtMin := -vect.Cross(segment.Tn, segment.Ta)
	dtMax := -vect.Cross(segment.Tn, segment.Tb)

	// Decision tree to decide which feature of the segment to collide with.
	if dt < dtMin {
		if dt < (dtMin - rsum) {
			return 0
		} else {
			return segmentEncapQuery(circle.Tc, segment.Ta, circle.Radius, segment.Radius, contacts[0], segment.A_tangent)
		}
	} else {
		if dt < dtMax {
			n := segment.Tn
			if dn >= 0.0 {
				n.Mult(-1)
			}
			con := contacts[0]
			pos := vect.Add(circle.Tc, vect.Mult(n, circle.Radius+dist*0.5))
			con.reset(pos, n, dist, 0)
			return 1
		} else {
			if dt < (dtMax + rsum) {
				return segmentEncapQuery(circle.Tc, segment.Tb, circle.Radius, segment.Radius, contacts[0], segment.B_tangent)
			} else {
				return 0
			}
		}
	}
	panic("Never reached")
}

func circle2polyFunc(contacts []*Contact, circle *CircleShape, poly *PolygonShape) int {

	axes := poly.TAxes

	mini := 0
	min := vect.Dot(axes[0].N, circle.Tc) - axes[0].D - circle.Radius
	for i, axis := range axes {
		dist := vect.Dot(axis.N, circle.Tc) - axis.D - circle.Radius
		if dist > 0.0 {
			return 0
		} else if dist > min {
			min = dist
			mini = i
		}
	}

	n := axes[mini].N
	a := poly.TVerts[mini]
	b := poly.TVerts[(mini+1)%poly.NumVerts]
	dta := vect.Cross(n, a)
	dtb := vect.Cross(n, b)
	dt := vect.Cross(n, circle.Tc)

	if dt < dtb {
		return circle2circleQuery(circle.Tc, b, circle.Radius, 0.0, contacts[0])
	} else if dt < dta {
		contacts[0].reset(
			vect.Sub(circle.Tc, vect.Mult(n, circle.Radius+min/2.0)),
			vect.Mult(n, -1),
			min,
			0,
		)
		return 1
	} else {
		return circle2circleQuery(circle.Tc, a, circle.Radius, 0.0, contacts[0])
	}
	panic("Never reached")
}

func poly2polyFunc(contacts []*Contact, poly1, poly2 *PolygonShape) int {
	min1, mini1 := findMSA(poly2, poly1.TAxes, poly1.NumVerts)
	if mini1 == -1 {
		return 0
	}

	min2, mini2 := findMSA(poly1, poly2.TAxes, poly2.NumVerts)
	if mini2 == -1 {
		return 0
	}

	// There is overlap, find the penetrating verts
	if min1 > min2 {
		return findVerts(contacts, poly1, poly2, poly1.TAxes[mini1].N, min1)
	} else {
		return findVerts(contacts, poly1, poly2, vect.Mult(poly2.TAxes[mini2].N, -1), min2)
	}

	panic("Never reached")
}

func findMSA(poly *PolygonShape, axes []PolygonAxis, num int) (min_out vect.Float, min_index int) {

	min := poly.valueOnAxis(axes[0].N, axes[0].D)
	if min > 0.0 {
		return 0, -1
	}

	for i := 1; i < num; i++ {
		dist := poly.valueOnAxis(axes[i].N, axes[i].D)
		if dist > 0.0 {
			return 0, -1
		} else if dist > min {
			min = dist
			min_index = i
		}
	}

	return min, min_index
}

func (poly *PolygonShape) valueOnAxis(n vect.Vect, d vect.Float) vect.Float {
	verts := poly.TVerts
	min := vect.Dot(n, verts[0])

	for i := 1; i < poly.NumVerts; i++ {
		min = vect.FMin(min, vect.Dot(n, verts[i]))
	}
	//fmt.Println(min, d)
	return min - d
}

func nextContact(contacts []*Contact, numPtr *int) *Contact {
	index := *numPtr

	if index < MaxPoints {
		*numPtr = index + 1
		return contacts[index]
	} else {
		return contacts[MaxPoints-1]
	}
	panic("Never reached")
}

func findVerts(contacts []*Contact, poly1, poly2 *PolygonShape, n vect.Vect, dist vect.Float) int {
	num := 0

	for i, v := range poly1.TVerts {
		if poly2.ContainsVert(v) {
			c := nextContact(contacts, &num)
			c.reset(v, n, dist, hashPair(poly1.Shape.Hash(), HashValue(i)))
		}
	}

	for i, v := range poly2.TVerts {
		if poly1.ContainsVert(v) {
			nextContact(contacts, &num).reset(v, n, dist, hashPair(poly2.Shape.Hash(), HashValue(i)))
		}
	}

	if num > 0 {
		return num
	} else {
		return findVertsFallback(contacts, poly1, poly2, n, dist)
	}

	panic("Never reached")
}

func findVertsFallback(contacts []*Contact, poly1, poly2 *PolygonShape, n vect.Vect, dist vect.Float) int {
	num := 0

	for i, v := range poly1.TVerts {
		if poly2.ContainsVertPartial(v, vect.Mult(n, -1)) {
			c := nextContact(contacts, &num)
			c.reset(v, n, dist, hashPair(poly1.Shape.Hash(), HashValue(i)))
		}
	}

	for i, v := range poly2.TVerts {
		if poly1.ContainsVertPartial(v, n) {
			nextContact(contacts, &num).reset(v, n, dist, hashPair(poly2.Shape.Hash(), HashValue(i)))
		}
	}

	return num
}

func segValueOnAxis(seg *SegmentShape, n vect.Vect, d vect.Float) vect.Float {
	a := vect.Dot(n, seg.Ta) - seg.Radius
	b := vect.Dot(n, seg.Tb) - seg.Radius
	return vect.FMin(a, b) - d
}

func findPoinsBehindSeg(contacts []*Contact, num *int, seg *SegmentShape, poly *PolygonShape, pDist, coef vect.Float) {
	dta := vect.Cross(seg.Tn, seg.Ta)
	dtb := vect.Cross(seg.Tn, seg.Tb)
	n := vect.Mult(seg.Tn, coef)

	for i := 0; i < poly.NumVerts; i++ {
		v := poly.TVerts[i]
		if vect.Dot(v, n) < vect.Dot(seg.Tn, seg.Ta)*coef+seg.Radius {
			dt := vect.Cross(seg.Tn, v)
			if dta >= dt && dt >= dtb {
				nextContact(contacts, num).reset(v, n, pDist, hashPair(poly.Shape.Hash(), HashValue(i)))
			}
		}
	}
}

func seg2polyFunc(contacts []*Contact, seg *SegmentShape, poly *PolygonShape) int {
	axes := poly.TAxes

	segD := vect.Dot(seg.Tn, seg.Ta)
	minNorm := poly.ValueOnAxis(seg.Tn, segD) - seg.Radius
	minNeg := poly.ValueOnAxis(vect.Mult(seg.Tn, -1), -segD) - seg.Radius
	if minNeg > 0.0 || minNorm > 0.0 {
		return 0
	}

	mini := 0
	poly_min := segValueOnAxis(seg, axes[0].N, axes[0].D)
	if poly_min > 0.0 {
		return 0
	}

	for i := 0; i < poly.NumVerts; i++ {
		dist := segValueOnAxis(seg, axes[i].N, axes[i].D)
		if dist > 0.0 {
			return 0
		} else if dist > poly_min {
			poly_min = dist
			mini = i
		}
	}

	num := 0

	poly_n := vect.Mult(axes[mini].N, -1)

	va := vect.Add(seg.Ta, vect.Mult(poly_n, seg.Radius))
	vb := vect.Add(seg.Tb, vect.Mult(poly_n, seg.Radius))
	if poly.ContainsVert(va) {
		nextContact(contacts, &num).reset(va, poly_n, poly_min, hashPair(seg.Shape.Hash(), 0))
	}
	if poly.ContainsVert(vb) {
		nextContact(contacts, &num).reset(vb, poly_n, poly_min, hashPair(seg.Shape.Hash(), 1))
	}

	if minNorm >= poly_min || minNeg >= poly_min {
		if minNorm > minNeg {
			findPoinsBehindSeg(contacts, &num, seg, poly, minNorm, 1.0)
		} else {
			findPoinsBehindSeg(contacts, &num, seg, poly, minNeg, -1.0)
		}
	}

	// If no other collision points are found, try colliding endpoints.
	if num == 0 {
		poly_a := poly.TVerts[mini]
		poly_b := poly.TVerts[(mini+1)%poly.NumVerts]

		if segmentEncapQuery(seg.Ta, poly_a, seg.Radius, 0.0, contacts[0], vect.Mult(seg.A_tangent, -1)) != 0 {
			return 1
		}
		if segmentEncapQuery(seg.Tb, poly_a, seg.Radius, 0.0, contacts[0], vect.Mult(seg.B_tangent, -1)) != 0 {
			return 1
		}
		if segmentEncapQuery(seg.Ta, poly_b, seg.Radius, 0.0, contacts[0], vect.Mult(seg.A_tangent, -1)) != 0 {
			return 1
		}
		if segmentEncapQuery(seg.Tb, poly_b, seg.Radius, 0.0, contacts[0], vect.Mult(seg.B_tangent, -1)) != 0 {
			return 1
		}
	}

	return num
}
