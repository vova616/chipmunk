package chipmunk

import (
	"github.com/vova616/chipmunk/transform"
	"github.com/vova616/chipmunk/vect"

	"log"
	//"fmt"
	"math"
)

type PolygonAxis struct {
	// The axis normal.
	N vect.Vect
	D vect.Float
}

type PolygonShape struct {
	Shape *Shape
	// The raw vertices of the polygon. Do not touch!
	// Use polygon.SetVerts() to change this.
	Verts Vertices
	// The transformed vertices. Do not touch!
	TVerts Vertices
	// The axes of the polygon. Do not touch!
	Axes []PolygonAxis
	// The transformed axes of the polygon Do not touch!
	TAxes []PolygonAxis
	// The number of vertices. Do not touch!
	NumVerts int
}

// Creates a new PolygonShape with the given vertices offset by offset.
// Returns nil if the given vertices are not valid.
func NewPolygon(verts Vertices, offset vect.Vect) *Shape {
	if verts == nil {
		log.Printf("Error: no vertices passed!")
		return nil
	}

	shape := newShape()
	poly := &PolygonShape{Shape: shape}

	poly.SetVerts(verts, offset)

	shape.ShapeClass = poly
	return shape
}

func (poly *PolygonShape) Moment(mass float32) vect.Float {

	sum1 := vect.Float(0)
	sum2 := vect.Float(0)

	println("using bad Moment calculation")
	offset := vect.Vect{0, 0}

	for i := 0; i < poly.NumVerts; i++ {

		v1 := vect.Add(poly.Verts[i], offset)
		v2 := vect.Add(poly.Verts[(i+1)%poly.NumVerts], offset)

		a := vect.Cross(v2, v1)
		b := vect.Dot(v1, v1) + vect.Dot(v1, v2) + vect.Dot(v2, v2)

		sum1 += a * b
		sum2 += a
	}

	return (vect.Float(mass) * sum1) / (6.0 * sum2)
}

// Sets the vertices offset by the offset and calculates the PolygonAxes.
func (poly *PolygonShape) SetVerts(verts Vertices, offset vect.Vect) {

	if verts == nil {
		log.Printf("Error: no vertices passed!")
		return
	}

	if verts.ValidatePolygon() == false {
		log.Printf("Warning: vertices not valid")
	}

	numVerts := len(verts)
	oldnumVerts := len(poly.Verts)
	poly.NumVerts = numVerts

	if oldnumVerts < numVerts {
		//create new slices
		poly.Verts = make(Vertices, numVerts)
		poly.TVerts = make(Vertices, numVerts)
		poly.Axes = make([]PolygonAxis, numVerts)
		poly.TAxes = make([]PolygonAxis, numVerts)

	} else {
		//reuse old slices
		poly.Verts = poly.Verts[:numVerts]
		poly.TVerts = poly.TVerts[:numVerts]
		poly.Axes = poly.Axes[:numVerts]
		poly.TAxes = poly.TAxes[:numVerts]
	}

	for i := 0; i < numVerts; i++ {
		a := vect.Add(offset, verts[i])
		b := vect.Add(offset, verts[(i+1)%numVerts])
		n := vect.Normalize(vect.Perp(vect.Sub(b, a)))

		poly.Verts[i] = a
		poly.Axes[i].N = n
		poly.Axes[i].D = vect.Dot(n, a)
	}
}

// Returns ShapeType_Polygon. Needed to implemet the ShapeClass interface.
func (poly *PolygonShape) ShapeType() ShapeType {
	return ShapeType_Polygon
}

func (poly *PolygonShape) Clone(s *Shape) ShapeClass {
	return poly.Clone2(s)
}

func (poly *PolygonShape) Clone2(s *Shape) *PolygonShape {
	clone := *poly
	clone.Verts = make(Vertices, len(poly.Verts))
	clone.TVerts = make(Vertices, len(poly.TVerts))
	clone.Axes = make([]PolygonAxis, len(poly.Axes))
	clone.TAxes = make([]PolygonAxis, len(poly.TAxes))

	clone.Verts = append(clone.Verts, poly.Verts...)
	clone.TVerts = append(clone.TVerts, poly.TVerts...)
	clone.Axes = append(clone.Axes, poly.Axes...)
	clone.TAxes = append(clone.TAxes, poly.TAxes...)

	clone.Shape = s

	return &clone
}

// Calculates the transformed vertices and axes and the bounding box.
func (poly *PolygonShape) update(xf transform.Transform) AABB {
	//transform axes
	{
		src := poly.Axes
		dst := poly.TAxes

		for i := 0; i < poly.NumVerts; i++ {
			n := xf.RotateVect(src[i].N)
			dst[i].N = n
			dst[i].D = vect.Dot(xf.Position, n) + src[i].D
		}
		/*
			fmt.Println("")
			fmt.Println("Started Axes")
			fmt.Println(xf.Rotation, xf.Position)
			for i:=0;i<poly.NumVerts;i++ {
				fmt.Println(src[i], dst[i])
			}
		*/
	}
	//transform verts
	{
		inf := vect.Float(math.Inf(1))
		aabb := AABB{
			Lower: vect.Vect{inf, inf},
			Upper: vect.Vect{-inf, -inf},
		}

		src := poly.Verts
		dst := poly.TVerts

		for i := 0; i < poly.NumVerts; i++ {
			v := xf.TransformVect(src[i])

			dst[i] = v
			aabb.Lower.X = vect.FMin(aabb.Lower.X, v.X)
			aabb.Upper.X = vect.FMax(aabb.Upper.X, v.X)
			aabb.Lower.Y = vect.FMin(aabb.Lower.Y, v.Y)
			aabb.Upper.Y = vect.FMax(aabb.Upper.Y, v.Y)
		}

		/*
			fmt.Println("Verts")
			for i:=0;i<poly.NumVerts;i++ {
				fmt.Println(src[i], dst[i])
			}
		*/
		return aabb
	}
}

// Returns true if the given point is located inside the box.
func (poly *PolygonShape) TestPoint(point vect.Vect) bool {
	return poly.ContainsVert(point)
}

func (poly *PolygonShape) ContainsVert(v vect.Vect) bool {
	for _, axis := range poly.TAxes {
		dist := vect.Dot(axis.N, v) - axis.D
		if dist > 0.0 {
			return false
		}
	}

	return true
}

func (poly *PolygonShape) ContainsVertPartial(v, n vect.Vect) bool {
	for _, axis := range poly.TAxes {
		if vect.Dot(axis.N, n) < 0.0 {
			continue
		}
		dist := vect.Dot(axis.N, v) - axis.D
		if dist > 0.0 {
			return false
		}
	}

	return true
}

func (poly *PolygonShape) ValueOnAxis(n vect.Vect, d vect.Float) vect.Float {
	verts := poly.TVerts
	min := vect.Dot(n, verts[0])

	for i := 1; i < poly.NumVerts; i++ {
		min = vect.FMin(min, vect.Dot(n, verts[i]))
	}

	return min - d
}
