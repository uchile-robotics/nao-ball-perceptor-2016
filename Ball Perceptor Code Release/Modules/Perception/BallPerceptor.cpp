/**
 * @file BallPerceptor.cpp
 * This file declares a module that provides a ball percept of the new oficial Robocup SPL ball.
 * @author Gabriel Azocar
 * @author Pablo Cano
 * UChile Robotics Team
 */

#include "BallPerceptor.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>

MAKE_MODULE(BallPerceptor, Perception)

// Alternate drawing macros that scale down if required
#define LINE2(n, x1, y1, x2, y2, w, s, c) LINE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define CROSS2(n, x, y, r, w, s, c) CROSS(n, scale(x), scale(y), scale(r), w, s, c)
#define CIRCLE2(n, x, y, r, w, s1, c1, s2, c2) CIRCLE(n, scale(x), scale(y), scale(r), w, s1, c1, s2, c2)
#define RECTANGLE3(n, x1, y1, x2, y2, w, s, c) RECTANGLE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define DRAWTEXT2(n, x1, y1, s, c, m) DRAWTEXT(n, scale(x1), scale(y1), s, c, m)
#define DOT2(n, x, y, c1, c2) DOT(n, scale(x), scale(y), c1, c2)

//Constructor
BallPerceptor::BallPerceptor()
{
  #ifndef TARGET_ROBOT
    gradient = 150;
  #endif
}

//Update Function
void BallPerceptor::update(BallPercept& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:NewBallPerceptor2:image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:NewBallPerceptor2:zdots", "drawingOnImage");
  scaleInput();
  
  //Calculate the image limits
  imageHeigth = theCameraInfo.height;
  imageWidth = theCameraInfo.width;

  searchBallFromBallSpots(ballPercept);

}

//Verification Functions
void BallPerceptor::searchBallFromBallSpots(BallPercept& ballPercept)
{
  std::vector<BallSpot> ballSpots = theBallSpots.ballSpots;
  ballPercept.balls.clear();
  ballPercept.status = BallPercept::notSeen;
  for (const BallSpot& ballSpot : ballSpots)
  {
    bool analyse = true;
    for (const BallPercept::Ball& ball : ballPercept.balls)
    {
      if ((Vector2<>(float(ballSpot.position.x), float(ballSpot.position.y)) - ball.positionInImage).abs() < (theCameraInfo.camera == CameraInfo::upper ? 4.f * ball.radiusInImage : ball.radiusInImage)) {
        analyse = false;
        break;
      }
    }
    
    if (analyse)
    {
      if (analyzeBallSpot(ballSpot, ballPercept))
      {
        BallPercept::Ball ball;
        ball.positionInImage = center;
        ball.radiusInImage = radius;
        calculateBallOnField(ball);
        if(checkBallOnField(ballPercept, ball))
        {
          drawBall(center);
          scaleOutput(ball);
          ballPercept.balls.push_back(ball);
        }
        else
        {
          drawError(ballSpot, "checkBallOnField");
        }
      }
    }
  }
  if (ballPercept.balls.size() != 0) {
    ballPercept.status = BallPercept::seen;
  }
  else
  {
    BallPercept::Ball ball;
    ball.positionInImage.x = 0;
    ball.positionInImage.y = 0;
    ball.radiusInImage = 0;
    calculateBallOnField(ball);
    ballPercept.balls.push_back(ball);
  }
}

bool BallPerceptor::analyzeBallSpot(const BallSpot ballSpot, BallPercept& ballPercept)
{
  ballValidity = false;
  bool ans = false;
  
  ans = !checkBallSpot(ballSpot) ? drawError(ballSpot, "checkBallSpot") :
  !checkRegionSizes(ballSpot) ? drawError(ballSpot,"checkRegionSize") :
  !searchEdgePoints(ballSpot, ColorRGBA(255, 0, 0, 0)) ? drawError(ballSpot, "searchEdgePoints") :
  !isBallFromPoints() ? drawError(ballSpot, "isBallFromPoints") :
  !checkNewRadius() ? drawError(ballSpot, "checkNewRadius") :
  !checkBallNear(ballPercept) ? drawError(ballSpot, "nearBall") :
  !searchEdgePoints(BallSpot(int(center.x + 0.5f), int(center.y + 0.5f)), ColorRGBA(0, 0, 255, 50)) ? drawError(ballSpot, "searchEdgePoints2") :
  !isBallFromPoints() ? drawError(ballSpot, "isBallFromPoints2") :
  !isRobotNear() ? drawError(ballSpot,"isRobotNear") :
  !checkNewRadius() ? drawError(ballSpot, "checkNewRadius") :
  !searchValidEdges() ? drawError(ballSpot, "validEdges") :
  !checkGreenInside() ? drawError(ballSpot, "checkGreenInside") :
  !ballValidity ? drawError(ballSpot, "BallValidity") :
  !checkPentagons() ? drawError(ballSpot, "checkPentagon()") :
  true;
  
  return ans;
}

bool BallPerceptor::calculateBallOnField(BallPercept::Ball& ballPercept) const
{
  const Vector2<> correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
  Vector3<> cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedCenter.x, theCameraInfo.opticalCenter.y - correctedCenter.y);
  cameraToBall.normalize(theFieldDimensions.ballRadius * theCameraInfo.focalLength / ballPercept.radiusInImage);
  Vector3<> rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
  const Vector3<> sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
  const Vector3<> bearingBasedCenterOnField = theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z - theFieldDimensions.ballRadius) / rotatedCameraToBall.z);
  
  if (rotatedCameraToBall.z < 0)
  {
    ballPercept.relativePositionOnField.x = bearingBasedCenterOnField.x;
    ballPercept.relativePositionOnField.y = bearingBasedCenterOnField.y;
  }
  else
  {
    ballPercept.relativePositionOnField.x = sizeBasedCenterOnField.x;
    ballPercept.relativePositionOnField.y = sizeBasedCenterOnField.y;
  }
  return true;
}

bool BallPerceptor::checkBallOnField(BallPercept& ballPercept, BallPercept::Ball ball) const
{
  // Not sure about self-localization => Do not use it to check ball position
  if(theRobotPose.validity < 1.f)
  {
    return true;
  }
  // Check, if the computed ball position is still on the carpet
  Pose2D currentRobotPose = theRobotPose + theOdometer.odometryOffset;
  Vector2<> absoluteBallPosition = (currentRobotPose * ball.relativePositionOnField);
  return ((fabs(absoluteBallPosition.x) < theFieldDimensions.xPosOpponentFieldBorder + 300.f) &&
          (fabs(absoluteBallPosition.y) < theFieldDimensions.yPosLeftFieldBorder + 300.f));
}

//Cascade Functions
bool BallPerceptor::checkBallSpot(const BallSpot& ballSpot)
{
  // Calculate an approximation of the radius based on bearing distance of the ball spot
  const Vector2<int>& spot = ballSpot.position;
  Vector2<> correctedStart = theImageCoordinateSystem.toCorrected(spot);
  Vector3<> cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x - correctedStart.x, theCameraInfo.opticalCenter.y - correctedStart.y);
  Vector3<> unscaledField = theCameraMatrix.rotation * cameraToStart;
  if(unscaledField.z >= 0.f)
  {
    return false; // Above horizon
  }
  const float scaleFactor = (theCameraMatrix.translation.z - theFieldDimensions.ballRadius) / unscaledField.z;
  cameraToStart *= scaleFactor;
  unscaledField *= scaleFactor;
  cameraToStart.y += cameraToStart.y > 0 ? -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
  cameraToStart /= scaleFactor;
  approxRadius = std::abs(theImageCoordinateSystem.fromCorrectedApprox(Vector2<int>(int(theCameraInfo.opticalCenter.x - cameraToStart.y), int(theCameraInfo.opticalCenter.y - cameraToStart.z))).x - spot.x);
  
  return true;
}

bool BallPerceptor::checkRegionSizes(const BallSpot& ballSpot)
{
  return ballSpot.height / (2 * approxRadius) <= regionSizeTolerance &&
		ballSpot.width / (2 * approxRadius) <= regionSizeTolerance;
}

bool BallPerceptor::searchEdgePoints(const BallSpot& ballSpot, const ColorRGBA& color)
{
  int currentIndex = 0;
  int ballSpotYBoundary = theFieldBoundary.getBoundaryY(ballSpot.position.x);
  ballSpotYBoundary = std::max(0, 2);
  validBallPoints = 0;
  for (float angle = 0; angle <= piValor * 2; angle += pointsRotation)
  {
    int actualX = 0, actualY = 0;
    int numberOfGreenPixels = 0;
    BallPoint newBallPoint;
    ballPoints[currentIndex] = newBallPoint;
    for (float actualLength = 1; actualLength < approxRadius * scanTolerance + 9; actualLength++)
    {
      actualX = ballSpot.position.x + int(cos(angle) * actualLength);
      actualY = ballSpot.position.y + int(sin(angle) * actualLength);
      if (actualX <= 0 || actualY <= ballSpotYBoundary || actualY >= imageHeigth || actualX >= imageWidth)
        break;
      const Image::Pixel pPixel = getPixel(actualY, actualX);
      if (theColorTable[pPixel].is(ColorClasses::green))
      {
        numberOfGreenPixels++;
        if (numberOfGreenPixels >= approxRadius / 4.f)
        {
          newBallPoint.isValid = true;
          newBallPoint.point.x = ballSpot.position.x + int(cos(angle) * (actualLength - numberOfGreenPixels) + 0.5f);
          newBallPoint.point.y = ballSpot.position.y + int(sin(angle) * (actualLength - numberOfGreenPixels) + 0.5f);
          ballPoints[currentIndex] = newBallPoint;
          validBallPoints++;
          break;
        }
      }
    }
    
    currentIndex++;
  }
  
  if (validBallPoints <= searchEdgePointsTolerance)
  {
    return true;
  }
  
  return true;
}

bool BallPerceptor::isBallFromPoints()
{
  float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;
  
  for (const BallPoint* ballPoint = ballPoints, *end = ballPoints + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
    if (ballPoint->isValid)
    {
      float x = static_cast<float>(ballPoint->point.x);
      float y = static_cast<float>(ballPoint->point.y);
      float xx = x * x;
      float yy = y * y;
      float z = xx + yy;
      Mx += x;
      My += y;
      Mxx += xx;
      Myy += yy;
      Mxy += x * y;
      Mz += z;
      Mxz += x * z;
      Myz += y * z;
    }
  
  // Construct and solve matrix
  // Result will be center and radius of ball in theImage.
  Matrix<3, 3> M(
                 Vector<3>(Mxx, Mxy, Mx),
                 Vector<3>(Mxy, Myy, My),
                 Vector<3>(Mx, My, static_cast<float>(validBallPoints)));
  Vector<3> v(-Mxz, -Myz, -Mz);
  
  Vector<3> BCD;
  if (!M.solve(v, BCD))
  {
    return false;
  }
  
  center.x = BCD[0] * -0.5f;
  center.y = BCD[1] * -0.5f;
  float radicand = BCD[0] * BCD[0] / 4.0f + BCD[1] * BCD[1] / 4.0f - BCD[2];
  if (radicand <= 3.f)
  {
    return false;
  }
  radius = std::sqrt(radicand);
  return true;
}

bool BallPerceptor::checkNewRadius()
{
  return radius / approxRadius >= downRadiusTolerance && radius / approxRadius <= upRadiusTolerance;
}

bool BallPerceptor::checkBallNear(BallPercept& ballPercept)
{
  for (const BallPercept::Ball& ball : ballPercept.balls) {
    if ((center - ball.positionInImage).abs() < ball.radiusInImage) {
      return false;
    }
  }
  return true;
}

bool BallPerceptor::isRobotNear()
{
  float centerX = theCameraInfo.camera == CameraInfo::upper ? center.x / 2 : center.x;
  float centerY = theCameraInfo.camera == CameraInfo::upper ? center.y / 2 : center.y;

  for (int i = 0; i < int(theRobotPercept.robots.size()); i++)
  {
    if (theRobotPercept.robots[i].x1 <= centerX - radius && centerX + radius <= theRobotPercept.robots[i].x2)
    {
      if (theRobotPercept.robots[i].y1 <= centerY - radius && centerY + radius <= theRobotPercept.robots[i].y2)
      {
        return false;
      }
    }
  }
  return true;
}

bool BallPerceptor::searchValidEdges()
{
  int validEdges = 0;
  for (float angle = 0; angle <= piValor * 2; angle += pointsRotation)
  {
    unsigned char lastY = 0xff;
    bool edge = false;
    int actualX;
    int actualY;
    for (float actualLength = radius * 0.8f; actualLength < radius * 1.2f; actualLength++)
    {
      actualX = int(center.x + cos(angle) * actualLength + 0.5f);
      actualY = int(center.y + sin(angle) * actualLength + 0.5f);
      
      if (actualX <= 0 || actualY <= 0 || actualY >= imageHeigth || actualX >= imageWidth)
        break;
      
      const Image::Pixel pPixel = getPixel(actualY, actualX);
      if (lastY != 0xff)
      {
        if (lastY != theColorTable[pPixel].colors)
        {
          edge = true;
          validEdges++;
          break;
        }
      }
      
      else if(theColorTable[pPixel].is(ColorClasses::green))
        break;
      
      lastY = theColorTable[pPixel].colors;
    }
  }
  
  if (validEdges < minValidEdges)
  {
    return false;
  }
  
  return true;
}

bool BallPerceptor::checkGreenInside()
{
  float numberOfGreenPixels = 0;
  float totalPixels = 0;
  int numOfTransitions = 0;
  bool body = theCameraInfo.camera == CameraInfo::lower;
  
  for (float angle = 0; angle <= piValor * 2; angle += pointsRotation)
  {
    unsigned char lastY = 0xff;
    for (float actualLength = 1; actualLength < radius - 1; actualLength++)
    {
      double actualX = center.x + cos(angle) * actualLength;
      double actualY = center.y + sin(angle) * actualLength;
      
      if (actualX < 0 || actualY < 0 || actualY > imageHeigth || actualX > imageWidth)
        break;
      
      if(body && !theBodyContour.isValidPoint(Vector2<int>((int)actualX, (int)actualY)))
        break;
      
      totalPixels++;
      const Image::Pixel pPixel = getPixel(int(actualY + 0.5f) , int(actualX + 0.5f));
      
      if (theColorTable[pPixel].is(ColorClasses::green))
      {
        numberOfGreenPixels++;
      }
      
      else
      {
        if (lastY != 0xff)
        {
          if (lastY != theColorTable[pPixel].colors)
          {
            numOfTransitions++;
          }
        }
        lastY = theColorTable[pPixel].colors;
      }
    }
  }
  
  if (numOfTransitions >= minNumOfTransitions*radius)
    ballValidity = true;
  
  if (numberOfGreenPixels/totalPixels < greenThrld)
    return true;
  
  return false;
}

bool BallPerceptor::checkPentagons()
{
  regions.clear();
  float gap = radius + radius/3.f;
  int step = int(gap/10.f + 0.5f);
  for (int i = 0; i < 20; i++) {
    Vector2<int> init((int)(center.x - gap + step*i),int(center.y - gap));
    Vector2<int> end(int(center.x - gap + step*i),int(center.y + gap));
    scanLine(Vector2<int>(0,2), init, end);
  }
  for (int i = 0; i < 20; i++) {
    Vector2<int> init((int)(center.x - gap),int(center.y - gap + step*i));
    Vector2<int> end(int(center.x + gap),int(center.y - gap + step*i));
    scanLine(Vector2<int>(2,0), init, end);
  }
  mergeRegions();
  
  int numOfPentagons = createPentagons();
  
  if((center - pentagonsCenter/float(numOfPentagons)).abs() > radius*radiusGap )
    return false;
  
  for (int j = 0; j < numOfPentagons; j++)
  {
    if (pentagons[j].valid)
    {
      if ((center - Vector2<>(float(pentagons[j].center.x), float(pentagons[j].center.y))).abs() > radius)
      {
        return false;
      }
    }
  }
  
  if (numOfPentagons <= 1) {
    return false;
  }
 
  
  int numOfUnions = validatePentagons();
  if (numOfUnions < numOfPentagons - 1) {
    return false;
  }
  
  return true;
}

//Pentagon Functions
void BallPerceptor::scanLine(Vector2<int> step, Vector2<int> init, Vector2<int> end)
{
  
  while (init.x < 0 || init.x > imageWidth || init.y < 0 || init.y > imageHeigth) {
    init += step;
    if(init.x > end.x || init.y > end.y)
      return;
  }
  bool hasInit = false;
  Vector2<int> init2, end2,origin = init;
  bool horizontal = step.y == 0;
  int lastY = getPixel(origin.y, origin.x).y;
  int comp = int(2.f*radius/3.f + 0.5f);
  int min = int(radius / 5.f + 0.5f);
  while (origin.x <= end.x && origin.y <= end.y) {
    if (origin.x < 0 || origin.x > imageWidth || origin.y < 0 || origin.y > imageHeigth)
      return;
    if (std::abs(lastY - getPixel(origin.y, origin.x).y) > gradient)
    {
      if (lastY > getPixel(origin.y, origin.x).y)
      {
        hasInit = true;
        init2 = origin;
        DOT2("module:NewBallPerceptor2:zdots", origin.x, origin.y, ColorRGBA::green, ColorRGBA::green);
      }
      else
      {
        end2 = origin;
        DOT2("module:NewBallPerceptor2:zdots", origin.x, origin.y, ColorRGBA::red, ColorRGBA::red);
        if (hasInit && ((horizontal && end2.x - init2.x < comp &&  end2.x - init2.x > min) || (!horizontal && end2.y - init2.y < comp &&  end2.y - init2.y > min))) {
          LINE2("module:NewBallPerceptor2:image", init2.x, init2.y, end2.x, end2.y, 1, Drawings::ps_solid, ColorRGBA::violet);
          Region r = { horizontal, init2, end2, -1 };
          regions.push_back(r);
        }
        hasInit = false;
      }
    }
    lastY = getPixel(origin.y, origin.x).y;

    origin += step;
  }
}

void BallPerceptor::mergeRegions()
{
  pentagons.clear();
  int index = 0;
  if (regions.size() < 2) {
    return;
  }
  for (std::vector<Region>::iterator i = regions.begin(); i != regions.end() - 1; i++) {
    for (std::vector<Region>::iterator j = i + 1; j != regions.end(); j++) {
      if (i->areIntersected(*j)) {
        if (i->index == -1 && j->index == -1) {
          pentagons.push_back(Pentagon());
          i->index = index;
          j->index = index++;
        }
        else if (i->index == -1)
          i->index = j->index;
        else if (j->index == -1)
          j->index = i->index;
        else
          changeIndex(j->index, i->index);
      }
    }
  }
}

void BallPerceptor::changeIndex(int from, int to)
{
  for (Region& region : regions) {
    if (region.index == from) {
      region.index = to;
    }
  }
}

int BallPerceptor::createPentagons()
{
  for (auto& region : regions) {
    if (region.index < 0) {
      continue;
    }
    pentagons[region.index].addRegion(region);

  }
  int num = 0;
  pentagonsCenter = Vector2<>();
  for (Pentagon& pentagon : pentagons)
  {
    if (pentagon.points == 0) {
      pentagon.valid = false;
      continue;
    }
    pentagon.setCenter();
    pentagon.isPentagon(radius*radius / areaFactor, radius);
    if(pentagon.valid)
    {
      pentagonsCenter += Vector2<>(float(pentagon.center.x),float(pentagon.center.y));
      num++;
      LINE2("module:NewBallPerceptor2:image", pentagon.width.x, pentagon.center.y, pentagon.width.y, pentagon.center.y, 1, Drawings::ps_solid, ColorRGBA::red);
      LINE2("module:NewBallPerceptor2:image", pentagon.center.x, pentagon.heigth.x, pentagon.center.x, pentagon.heigth.y, 1, Drawings::ps_solid, ColorRGBA::red);
      CROSS2("module:NewBallPerceptor2:image", pentagon.center.x, pentagon.center.y, 1, 1, Drawings::ps_solid, ColorRGBA::blue);
    }
  }
  return num;
}

int BallPerceptor::validatePentagons()
{
  int num = 0;
  for (std::vector<Pentagon>::iterator i = pentagons.begin(); i != pentagons.end(); i++) {
    if (!i->valid) {
      continue;
    }
    for (std::vector<Pentagon>::iterator j = i; j != pentagons.end(); j++) {
      if (!j->valid) {
        continue;
      }
      if (i != j) {
        float dist = float((i->center - j->center).abs());
		if (dist < radius + radius*radiusGap && dist > radius - radius*radiusGap) {
          num++;
          LINE2("module:NewBallPerceptor2:image", i->center.x, i->center.y, j->center.x, j->center.y, 1, Drawings::ps_solid, ColorRGBA::red);
        }
      }
    }
  }
  return num;
}

//Debug Functions
void BallPerceptor::drawBall(const Vector2<float>& pos) const
{
  CIRCLE2("module:NewBallPerceptor2:image", pos.x, pos.y, radius, 0.5, Drawings::ps_solid, ColorRGBA::black, Drawings::bs_solid, ColorRGBA(255, 128, 64, 100));
}

bool BallPerceptor::drawError(BallSpot ballSpot, std::string message)
{
  CROSS2("module:NewBallPerceptor2:image",ballSpot.position.x, ballSpot.position.y,1, 1, Drawings::ps_solid, ColorRGBA::black);
  DRAWTEXT2("module:NewBallPerceptor2:image", ballSpot.position.x + 3, ballSpot.position.y + 2, 5, ColorRGBA::black, message);
  return false;
}

bool BallPerceptor::showRegionSizes(const BallSpot& ballSpot)
{
  float heightLength = ballSpot.height / 2;
  float widthLength = ballSpot.width / 2;
  drawError(ballSpot, "checkRegionSizes");
  LINE2("module:NewBallPerceptor2:image", ballSpot.position.x, ballSpot.position.y, ballSpot.position.x + widthLength, ballSpot.position.y, 1, Drawings::ps_solid, ColorRGBA::violet);
  LINE2("module:NewBallPerceptor2:image", ballSpot.position.x, ballSpot.position.y, ballSpot.position.x - widthLength, ballSpot.position.y, 1, Drawings::ps_solid, ColorRGBA::violet);
  LINE2("module:NewBallPerceptor2:image", ballSpot.position.x, ballSpot.position.y, ballSpot.position.x, ballSpot.position.y + heightLength, 1, Drawings::ps_solid, ColorRGBA::violet);
  LINE2("module:NewBallPerceptor2:image", ballSpot.position.x, ballSpot.position.y, ballSpot.position.x, ballSpot.position.y - heightLength, 1, Drawings::ps_solid, ColorRGBA::violet);
  return false;
}

//Scale Functions
void BallPerceptorScaler::scaleInput()
{
  typedef BallPerceptorBase B;
  theCameraInfo = B::theCameraInfo;
  theBallSpots = B::theBallSpots;

  //Do not copy "table"
  theImageCoordinateSystem.rotation = B::theImageCoordinateSystem.rotation;
  theImageCoordinateSystem.invRotation = B::theImageCoordinateSystem.invRotation;
  theImageCoordinateSystem.origin = B::theImageCoordinateSystem.origin;
  theImageCoordinateSystem.offset = B::theImageCoordinateSystem.offset;
  theImageCoordinateSystem.a = B::theImageCoordinateSystem.a;
  theImageCoordinateSystem.b = B::theImageCoordinateSystem.b;

  if (theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
  {
    theCameraInfo.width *= 2;
    theCameraInfo.height *= 2;
    theCameraInfo.opticalCenter *= 2.f;
    theCameraInfo.focalLength *= 2.f;
    theCameraInfo.focalLengthInv /= 2.f;
    theCameraInfo.focalLenPow2 *= 4.f;

    theImageCoordinateSystem.origin *= 2.f;
    theImageCoordinateSystem.b *= 0.5f;

    for (BallSpot& b : theBallSpots.ballSpots)
    {
      b.position *= 2;
      ++b.position.x; //Original y channel was the second one
    }
  }
  theImageCoordinateSystem.setCameraInfo(theCameraInfo);
}

void BallPerceptorScaler::scaleOutput(BallPercept::Ball& ball) const
{
  if (theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
  {
    ball.positionInImage *= 0.5f;
    ball.radiusInImage *= 0.5f;
  }
}
