// Andrei Gaponenko, 2011

#include "Mu2eG4/inc/constructVisualizationRegions.hh"

#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include "G4Color.hh"
#include "G4LogicalVolume.hh"
#include "G4Box.hh"
#include "G4Transform3D.hh"

#include "CLHEP/Vector/ThreeVector.h"
#include "CLHEP/Vector/Rotation.h"
#include "CLHEP/Units/SystemOfUnits.h"

#include "cetlib/exception.h"

#include "GeometryService/inc/GeomHandle.hh"
#include "GeometryService/inc/WorldG4.hh"
#include "BFieldGeom/inc/BFieldManager.hh"

#include "G4Helper/inc/G4Helper.hh"
#include "G4Helper/inc/VolumeInfo.hh"
#include "G4Helper/inc/AntiLeakRegistry.hh"

#include "Mu2eUtilities/inc/SimpleConfig.hh"
#include "Mu2eG4/inc/nestBox.hh"
#include "Mu2eG4/inc/findMaterialOrThrow.hh"


#define AGDEBUG(stuff) std::cerr<<"AG: "<<__FILE__<<", line "<<__LINE__<<": "<<stuff<<std::endl;
//#define AGDEBUG(stuff)

namespace mu2e {

  //================================================================
  void constructVisualizationRegions(const VolumeInfo& worldVolume, const SimpleConfig& config)
  {
    GeomHandle<WorldG4> worldGeom;

    const bool forceAuxEdgeVisible = config.getBool("g4.forceAuxEdgeVisible",false);
    const bool doSurfaceCheck      = false; // overlaps are OK
    const bool placePV             = true;

    const int verbosityLevel = config.getInt("visregions.verbosityLevel", 0);

    //----------------------------------------------------------------
    // Field maps
    if(config.getBool("visregions.fieldMaps.visible", false)) {
      const bool solid = config.getBool("visregions.fieldMaps.solid");

      std::vector<double> red;   config.getVectorDouble("visregions.fieldMaps.color.red", red);
      if(red.empty()) {
        throw cet::exception("GEOM")<<"constructVisualizationRegions(): visregions.fieldMaps.color.red is empty";
      }
      std::vector<double> green; config.getVectorDouble("visregions.fieldMaps.color.green", green, red.size());
      std::vector<double> blue;  config.getVectorDouble("visregions.fieldMaps.color.blue", blue, red.size());

      G4Material *material = findMaterialOrThrow(config.getString("visregions.fieldMaps.material"));

      std::vector<double> squashY;
      config.getVectorDouble("visregions.fieldMaps.squashY", squashY, squashY);


      GeomHandle<BFieldManager> fieldMgr;

      const BFieldManager::MapContainerType& maps = fieldMgr->getMapContainer();
      int mapNumber(0);
      for(BFieldManager::MapContainerType::const_iterator i=maps.begin(); i!=maps.end(); ++i, ++mapNumber) {
        const BFMap& field = i->second;

        CLHEP::Hep3Vector mapCenterInMu2e(field.xmin() + field.dx()*(field.nx()-1)/2,

                                          squashY.empty()?
                                          field.ymin() + field.dy()*(field.ny()-1)/2
                                          : (squashY[1]+squashY[0])/2.,

                                          field.zmin() + field.dz()*(field.nz()-1)/2
                                          );

        std::vector<double> mapHalfSize(3);
        mapHalfSize[0] =  field.dx()*(field.nx()-1)/2.;
        mapHalfSize[1] =  squashY.empty() ? field.dy()*(field.ny()-1)/2. : (squashY[1]-squashY[0])/2.;
        mapHalfSize[2] =  field.dz()*(field.nz()-1)/2.;

        int icolor = mapNumber % red.size();
        G4Colour color(red[icolor], green[icolor], blue[icolor]);

        std::ostringstream boxname;
        boxname<<"VisualizationMapBox_"<<i->first;

        if(verbosityLevel) {
          std::cout<<__func__<<": adding "<<boxname.str()<<" = {";
          std::copy(mapHalfSize.begin(), mapHalfSize.end(), std::ostream_iterator<double>(std::cout, ", "));
          std::cout<<"} at "<<mapCenterInMu2e<<std::endl;
        }

        nestBox(boxname.str(),
                mapHalfSize,
                material,
                0,
                mapCenterInMu2e + worldGeom->mu2eOriginInWorld(),
                worldVolume, 0,
                true/*visible*/,
                color,
                solid,
                forceAuxEdgeVisible,
                placePV,
                doSurfaceCheck
                );

      }
    }

    //----------------------------------------------------------------
    // Add the arbitrary boxes

    if(config.getBool("visregions.boxes.visible", false)) {

      const bool solid(config.getBool("visregions.boxes.solid"));
      G4Material *material = findMaterialOrThrow(config.getString("visregions.boxes.material"));

      std::vector<double> red; config.getVectorDouble("visregions.boxes.color.red", red);
      if(red.empty()) {
        throw cet::exception("GEOM")<<"constructVisualizationRegions(): visregions.boxes.color.red is empty";
      }
      std::vector<double> green; config.getVectorDouble("visregions.boxes.color.green", green, red.size());
      std::vector<double> blue; config.getVectorDouble("visregions.boxes.color.blue", blue, red.size());

      std::vector<double> xmin; config.getVectorDouble("visregions.boxes.xmin", xmin);
      const int nboxes = xmin.size();
      std::vector<double> ymin; config.getVectorDouble("visregions.boxes.ymin", ymin, nboxes);
      std::vector<double> zmin; config.getVectorDouble("visregions.boxes.zmin", zmin, nboxes);

      std::vector<double> xmax; config.getVectorDouble("visregions.boxes.xmax", xmax, nboxes);
      std::vector<double> ymax; config.getVectorDouble("visregions.boxes.ymax", ymax, nboxes);
      std::vector<double> zmax; config.getVectorDouble("visregions.boxes.zmax", zmax, nboxes);

      for(int ibox = 0; ibox < nboxes; ++ibox) {

        CLHEP::Hep3Vector boxCenterInMu2e((xmax[ibox]+xmin[ibox])/2,
                                          (ymax[ibox]+ymin[ibox])/2,
                                          (zmax[ibox]+zmin[ibox])/2
                                          );

        std::vector<double> boxHalfSize(3);
        boxHalfSize[0] = (xmax[ibox]-xmin[ibox])/2;
        boxHalfSize[1] = (ymax[ibox]-ymin[ibox])/2;
        boxHalfSize[2] = (zmax[ibox]-zmin[ibox])/2;

        int icolor = ibox % red.size();
        G4Colour color(red[icolor], green[icolor], blue[icolor]);

        std::ostringstream boxname;
        boxname<<"VisualizationBox"<<std::setw(3)<<std::setfill('0')<<ibox;

        if(verbosityLevel) {
          std::cout<<__func__<<": adding "<<boxname.str()<<" = {";
          std::copy(boxHalfSize.begin(), boxHalfSize.end(), std::ostream_iterator<double>(std::cout, ", "));
          std::cout<<"} at "<<boxCenterInMu2e<<std::endl;
        }

        nestBox(boxname.str(),
                boxHalfSize,
                material,
                0,
                boxCenterInMu2e + worldGeom->mu2eOriginInWorld(),
                worldVolume, 0,
                true/*visible*/,
                color,
                solid,
                forceAuxEdgeVisible,
                placePV,
                doSurfaceCheck
                );
      }
    }
  }

} // namespace mu2e
