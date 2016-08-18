/**
 * @file BallPercept.cpp
 *
 * Very simple representation of a seen ball
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#include "BallPercept.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void BallPercept::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:BallPercept:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:BallPercept", "robot");
  TRANSLATE3D("representation:BallPercept", 0, 0, -230);
  if (status == seen) {
    for (const Ball& ball : balls) {
      CIRCLE("representation:BallPercept:Image",
             ball.positionInImage.x,
             ball.positionInImage.y,
             ball.radiusInImage,
             1,
             Drawings::ps_solid,
             ColorRGBA::black,
             Drawings::bs_solid,
             ColorRGBA(255, 128, 64, 100));
      CIRCLE("representation:BallPercept:Field",
             ball.relativePositionOnField.x,
             ball.relativePositionOnField.y,
             50,
             3,
             Drawings::ps_solid,
             ColorRGBA::orange,
             Drawings::bs_null,
             ColorRGBA::orange);
      SPHERE3D("representation:BallPercept", ball.relativePositionOnField.x, ball.relativePositionOnField.y, 35, 35, ColorRGBA::orange);
    }
  }
}
