package chipmunk

import ( 
	"github.com/vova616/chipmunk/vect"
)

// Wrapper around []vect.Vect.
type Vertices []vect.Vect

// Checks if verts forms a valid polygon.
// The vertices must be convex and winded clockwise.
func (verts Vertices) ValidatePolygon() bool {
	numVerts := len(verts)
	for i := 0; i < numVerts; i++ {
		a := verts[i]
		b := verts[(i+1)%numVerts]
		c := verts[(i+2)%numVerts]

		if vect.Cross(vect.Sub(b, a), vect.Sub(c, b)) > 0.0 {
			return false
		}
	}

	return true
}
