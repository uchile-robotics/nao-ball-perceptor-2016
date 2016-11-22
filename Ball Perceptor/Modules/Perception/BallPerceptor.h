/**
* @file BallPerceptor.cpp
* This file declares a module that provides a ball percept of the new oficial Robocup SPL ball.
* @author Gabriel Azocar
* @author Pablo Cano
* UChile Robotics Team
*
* Partially based on B-Human's ball perceptor of the coderelease 2014
* whose authors are:
* @author Colin Graf
* @author marcel
* @author Florian Maaß
* @author Thomas Röfer
*/

#pragma once

#ifdef TARGET_ROBOT
#define IS_FULL_SIZE true
#else
#define IS_FULL_SIZE theImage.isFullSize
#endif

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/ColorTable.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ImageCoordinateSystem.h"
#include "Representations/Perception/BallPercept.h"
#include "Representations/Perception/RobotPercept.h"
#include "Representations/Perception/FieldBoundary.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/BallSpots.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/Math/BHMath.h"
#include <math.h>
#include <limits.h>

MODULE(BallPerceptor,
{ ,
  REQUIRES(BodyContour),
  REQUIRES(FieldDimensions),
  REQUIRES(Image),
  REQUIRES(FieldBoundary),
  REQUIRES(CameraMatrix),
  REQUIRES(ImageCoordinateSystem),
  REQUIRES(CameraInfo),
  REQUIRES(ColorTable),
  REQUIRES(Odometer),
  REQUIRES(BallSpots),
  REQUIRES(RobotPercept),
  USES(RobotPose),
  USES(BallModel),
  PROVIDES_WITH_MODIFY_AND_OUTPUT_AND_DRAW(BallPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(0.4f) pointsRotation,
    (float)(2) scanTolerance,
    (float)(1.8) upRadiusTolerance,
    (float)(0.6f) downRadiusTolerance,
    (int)(6) searchEdgePointsTolerance,
    (float)(0.2f) minNumOfTransitions,
    (float)(0.2f) greenThrld,
    (int)(20) gradient,
    (float)(1.8f) regionSizeTolerance,
    (int)(11) minValidEdges,
    float)(2.f) areaFactor,
    (float)(1.f) radiusGap,
  }),
});


/**
 * The class scales the input and output data if full size images
 * are avalable.
 */
class BallPerceptorScaler : public BallPerceptorBase
{
  
private:
  using BallPerceptorBase::theImage; // prevent direct access to image
  
protected:
  CameraInfo theCameraInfo;
  BallSpots theBallSpots;
  ImageCoordinateSystem theImageCoordinateSystem;
  
  /**
   * The only access to image pixels.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param The pixel as a temporary object. Do not use yCbCrPadding of that pixel.
   */
  const Image::Pixel getPixel(int y, int x) const
  {
    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
      return theImage.getFullSizePixel(y, x);
    else
      return theImage[y][x];
  }
  
  /**
   * Update the copies of input representations that contain data
   * that might have to be scaled up.
   * Must be called in each cycle before any computations are performed.
   */
  void scaleInput();
  
  /**
   * Scale down the output if required.
   * @param ballPercept The ball percept the fields of which might be scaled.
   */
  void scaleOutput(BallPercept::Ball& ball) const;
  
  /**
   * Scale down a single value if required.
   * This is only supposed to be used in debug drawings.
   * @param value The value that might be scaled.
   * @return The scaled value.
   */
  template<typename T> T scale(T value) const
  {
    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
      return value / (T) 2;
    else
      return value;
  }
};

class BallPerceptor : public BallPerceptorScaler
{
public:
  //Constructor
  BallPerceptor();
  
private:
  class BallPoint
  {
  public:
    Vector2<int> step;
    Vector2<int> start;
    Vector2<int> point;
    Vector2<> pointf;
    bool atBorder;
    bool isValid;
    
    BallPoint() : atBorder(false), isValid(false) {}
  };
  
  struct Region
  {
    bool horizontal;
    Vector2<int> init;
    Vector2<int> end;
    int index;
    
    bool areIntersected(Region& other)
    {
      if(horizontal == other.horizontal)
      {
        if(horizontal){
          return std::abs(init.y - other.init.y) < 3 && init.x <= other.end.x && end.x >= other.init.x;
        }
        else{
          return std::abs(init.x - other.init.x) < 3 && init.y <= other.end.y && end.y >= other.init.y;
        }
      }
      else{
        if(horizontal){
          return (other.init.x >= init.x && other.init.x < end.x) && (init.y >= other.init.y && init.y <= other.end.y);
        }
        else{
          return (init.x >= other.init.x && init.x < other.end.x) && (other.init.y >= init.y && other.init.y <= end.y);
        }
      }
    }
  };

  struct Pentagon
  {
    Pentagon() : width(INT_MAX,0), heigth(INT_MAX,0), points(0){}

    void addRegion(Region& region)
    {
      center += region.init + region.end;
      points += 2;

      width.x = region.init.x < width.x ? region.init.x : width.x;
      width.y = region.end.x > width.y ? region.end.x : width.y;

      heigth.x = region.init.y < heigth.x ? region.init.y : heigth.x;
      heigth.y = region.end.y > heigth.y ? region.end.y : heigth.y;
        
    }
    
    void isPentagon(float maxArea, float radius)
    {
      float totalHeigh = float(heigth.y - heigth.x);
      float totalWidth = float(width.y - width.x);
      if(totalHeigh > 2*radius/3 || totalWidth > 2*radius/3)
      {
         valid = false;
         return;
      }
      float factor = totalHeigh/totalWidth;
      if(factor < 0.3f || factor > 3.f)
      {
        valid = false;
        return;
      }
      area = totalWidth*totalHeigh;
      valid = area < maxArea;
    }

    void setCenter()
    {
      center = center / points;
    }
    
    Vector2<int> center;
    Vector2<int> width;
    Vector2<int> heigth;
    int points;
    float area;
    bool valid;
  };
  
	//Update Functions
	void update(BallPercept& ballPercept);

  //Verification Functions
  void searchBallFromBallSpots(BallPercept& ballPercept);
  bool analyzeBallSpot(const BallSpot ballSpot, BallPercept& ballPercept);
  bool calculateBallOnField(BallPercept::Ball& ballPercept) const;
  bool checkBallOnField(BallPercept& ballPercept, BallPercept::Ball ball) const;
   
  //Cascade Functions
	bool checkBallSpot(const BallSpot& ballSpot);
  bool checkRegionSizes(const BallSpot& ballSpot);
	bool searchEdgePoints(const BallSpot& ballSpot,const ColorRGBA& color);
  bool isBallFromPoints();
  bool checkNewRadius();
  bool checkBallNear(BallPercept& ballPercept);
  bool isRobotNear();
	bool searchValidEdges();
  bool checkGreenInside();
  bool checkPentagons();

  //Pentagon Functions
  void scanLine(Vector2<int> step, Vector2<int> origin, Vector2<int> end);
  void mergeRegions();
  void changeIndex(int from, int to);
  int createPentagons();
  int validatePentagons();
 
  //Debug Functions
	void drawBall(const Vector2<float>& pos) const;
	bool drawError(BallSpot ballSpot, std::string message);
	bool showRegionSizes(const BallSpot& ballSpot);
  
	// Internal Variables
	double imageHeigth, imageWidth, approxRadius;
  float radius;
	int validBallPoints;
	bool ballValidity;
	Vector2<float> center, pentagonsCenter;
	BallPoint ballPoints[16];
  std::vector<Region> regions;
  std::vector<Pentagon> pentagons;
};
