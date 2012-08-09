package transform

import (
	"encoding/json" 
	"github.com/vova616/chipmunk/vect"
	"log"
)

func (xf Transform) MarshalJSON() ([]byte, error) {
	xfData := struct {
		Position vect.Vect
		Rotation vect.Float
	}{
		Position: xf.Position,
		Rotation: xf.Angle(),
	}

	return json.Marshal(&xfData)
}

func (xf *Transform) UnmarshalJSON(data []byte) error {
	xf.SetIdentity()

	xfData := struct {
		Position *vect.Vect
		Rotation vect.Float
	}{
		Position: &xf.Position,
		Rotation: xf.Angle(),
	}

	err := json.Unmarshal(data, &xfData)
	if err != nil {
		log.Printf("Error decoding transform")
		return err
	}

	xf.Position = *xfData.Position
	xf.SetAngle(xfData.Rotation)

	return nil
}
