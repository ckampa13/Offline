//
// Class to perform BaBar Kalman fit
//
// $Id: KalFitHack.cc,v 1.4 2014/04/08 04:25:46 murat Exp $
// $Author: murat $ 
// $Date: 2014/04/08 04:25:46 $
//

// the following has to come before other BaBar includes
#include "BaBar/BaBar.hh"
#include "CalPatRec/inc/KalFitHack.hh"
#include "KalmanTests/inc/PanelAmbigResolver.hh"
#include "KalmanTests/inc/PocaAmbigResolver.hh"
#include "KalmanTests/inc/HitAmbigResolver.hh"
#include "CalPatRec/inc/HitAmbigResolverHack.hh"
#include "KalmanTests/inc/FixedAmbigResolver.hh"
#include "KalmanTests/inc/BaBarMu2eField.hh"
//geometry
#include "GeometryService/inc/GeometryService.hh"
#include "GeometryService/inc/GeomHandle.hh"
#include "GeometryService/inc/getTrackerOrThrow.hh"
#include "BFieldGeom/inc/BFieldConfig.hh"
// conditions
#include "ConditionsService/inc/ConditionsHandle.hh"
#include "ConditionsService/inc/TrackerCalibrations.hh"
// data
#include "RecoDataProducts/inc/StrawHitCollection.hh"
#include "RecoDataProducts/inc/StrawHit.hh"
// tracker
#include "TrackerGeom/inc/Tracker.hh"
#include "TrackerGeom/inc/Straw.hh"
// BaBar
#include "KalmanTrack/KalHit.hh"
#include "TrkBase/HelixTraj.hh"
//09 - 26 - 2013 gianipez added the following include file
#include "TrkBase/HelixParams.hh"
//----------------------------------------
#include "TrkBase/TrkHelixUtils.hh"
#include "TrkBase/TrkHotListFull.hh"
#include "TrkBase/TrkMomCalculator.hh"
#include "TrkBase/TrkPoca.hh"
#include "BaBar/ErrLog.hh"
#include "BField/BFieldFixed.hh"
#include "DetectorModel/DetIntersection.hh"
#include "DetectorModel/DetMaterial.hh"
// boost
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/weighted_variance.hpp>
// C++
#include <iostream>
#include <fstream>
#include <string>
#include <memory>

using namespace std; 

namespace mu2e 
{
// comparison functor for ordering hits
  struct fltlencomp : public binary_function<TrkStrawHit*, TrkStrawHit*, bool> {
    fltlencomp(TrkFitDirection::FitDirection fdir=TrkFitDirection::downstream) : _fdir(fdir) {}
    bool operator()(TrkStrawHit* x, TrkStrawHit* y) { 
      return _fdir == TrkFitDirection::downstream ? x->fltLen() < y->fltLen() : y->fltLen() < x->fltLen() ;
    }
    TrkFitDirection::FitDirection _fdir;
  };

  struct timecomp : public binary_function<TrkStrawHit*, TrkStrawHit*, bool> {
    timecomp() {}
    bool operator()(TrkStrawHit* x, TrkStrawHit* y) { 
      return x->hitT0()._t0 < y->hitT0()._t0;
    }
    TrkFitDirection::FitDirection _fdir;
  };
// construct from a parameter set  
  KalFitHack::KalFitHack(fhicl::ParameterSet const& pset) :
// KalFitHack parameters
    _debug(pset.get<int>("debugLevel",0)),
    _weedhits(pset.get<bool>("weedhits",true)),
    _maxhitchi(pset.get<double>("maxhitchi",4.0)),
    _maxweed(pset.get<unsigned>("maxweed",10)),
    _herr(pset.get< vector<double> >("hiterr")),
    _maxdriftpull(pset.get<double>("maxDriftPull",10)),
    // t0 parameters
    _initt0(pset.get<bool>("initT0",true)),
    _updatet0(pset.get<bool>("updateT0",true)),
    fMinHitDrift(pset.get<double>("HitMinDrift")),
    fRdriftMinusDocaTol(pset.get<double>("RdriftMinusDocaTol")),
    _daveMode(pset.get<int>("daveMode" ,0)),
    _t0tol(pset.get< vector<double> >("t0Tolerance")),
    _t0errfac(pset.get<double>("t0ErrorFactor",1.2)),
    _mint0doca(pset.get<double>("minT0DOCA",-0.2)),
    _t0nsig(pset.get<double>("t0window",2.5)),
    _dtoffset(pset.get<double>("dtOffset")),
    fScaleErrDoublet(pset.get<double>("scaleErrDoublet")),
    fMarkDoublets(0),
    fILoopMarkDoublets(pset.get<int>("iLoopMarkDoublets")),
    fMinDriftDoublet  (pset.get<double>("minDriftdoublet")),
    fDeltaDriftDoublet(pset.get<double>("deltaDriftDoublet")),
    //
    _removefailed(pset.get<bool>("RemoveFailedFits",true)),
    _minnstraws(pset.get<unsigned>("minnstraws",15)),
    _ambigstrategy(pset.get< vector<int> >("ambiguityStrategy")),
    _bfield(0),
    fNIter(0)
  {
    //    fStopwatch = new TStopwatch();
// set KalContext parameters
    _disttol = pset.get<double>("IterationTolerance",0.1);
    _intertol = pset.get<double>("IntersectionTolerance",100.0);
    _maxiter = pset.get<long>("MaxIterations",10);
    _maxinter = pset.get<long>("MaxIntersections",0);
    _matcorr = pset.get<bool>("materialCorrection",true);
    _fieldcorr = pset.get<bool>("fieldCorrection",false);
    _smearfactor = pset.get<double>("SeedSmear",1.0e6);
    _sitethresh = pset.get<double>("SiteMomThreshold",0.2);
    _momthresh = pset.get<double>("MomThreshold",10.0);
    _mingap = pset.get<double>("mingap",0.1);
    _minfltlen = pset.get<double>("MinFltLen",0.1);
    _minmom = pset.get<double>("MinMom",10.0);
    _fltepsilon = pset.get<double>("FltEpsilon",0.001);
    _divergeflt = pset.get<double>("DivergeFlt",1.0e3);
    _mindot = pset.get<double>("MinDot",0.0);
    _maxmomdiff = pset.get<double>("MaxMomDiff",0.5);
    _momfac = pset.get<double>("MomFactor",0.0);
    _maxpardif[0] = _maxpardif[1] = pset.get<double>("MaxParameterDifference",1.0);
    // DOF counting subdivision is illogical, FIXME!!!!
    _mindof[0] = _mindof[2] = pset.get<double>("MinNDOF",10);
    _mindof[1] = 0;
//----------------------------------------------------------------------
// 2015 - 01 -09 G. Pezzullo and P. Murat
// we noticed that with respect to KalmanTest/src/KalFit.cc we were using
// different ranges and divisions for the magnetic field
//----------------------------------------------------------------------
//     _bintconfig._maxRange = pset.get<double>("BFieldIntMaxRange",1.0e5); // 100 m
//     _bintconfig._intTolerance = pset.get<double>("BFieldIntTol",0.01); // 1 KeV
//     _bintconfig._intPathMin = pset.get<double>("BFieldIntMin",5.0); // 5 mm
//     _bintconfig._divTolerance = pset.get<double>("BFieldIntDivTol",0.01); // 10 KeV
//     _bintconfig._divPathMin = pset.get<double>("BFieldIntDivMin",10.0); // 10 mm
//     _bintconfig._divStepCeiling = pset.get<double>("BFieldIntDivMax",100.0); // 100 mm
    _bintconfig._maxRange = pset.get<double>("BFieldIntMaxRange",1.0e5); // 100 m
    _bintconfig._intTolerance = pset.get<double>("BFieldIntTol",0.01); // 10 KeV
    _bintconfig._intPathMin = pset.get<double>("BFieldIntMin",20.0); // 20 mm
    _bintconfig._divTolerance = pset.get<double>("BFieldIntDivTol",0.05); // 50 KeV
    _bintconfig._divPathMin = pset.get<double>("BFieldIntDivMin",50.0); // 50 mm
    _bintconfig._divStepCeiling = pset.get<double>("BFieldIntDivMax",500.0); // 500 mm
    // make sure we have at least one entry for additional errors
    if(_herr.size() <= 0) throw cet::exception("RECO")<<"mu2e::KalFitHack: no hit errors specified" << endl;
    if(_herr.size() != _ambigstrategy.size()) throw cet::exception("RECO")<<"mu2e::KalFitHack: inconsistent ambiguity resolution" << endl;
    if(_herr.size() != _t0tol.size()) throw cet::exception("RECO")<<"mu2e::KalFitHack: inconsistent ambiguity resolution" << endl;
    // construct the ambiguity resolvers
    for(size_t iambig=0;iambig<_ambigstrategy.size();++iambig){
      switch (_ambigstrategy[iambig] ){
	case fixedambig: default:
	  _ambigresolver.push_back(new FixedAmbigResolver(pset));
	  break;
	case hitambig:
	  _ambigresolver.push_back(new HitAmbigResolver(pset));
	  break;
	case panelambig:
	  _ambigresolver.push_back(new PanelAmbigResolver(pset));
	  break;
	case pocaambig:
	  _ambigresolver.push_back(new PocaAmbigResolver(pset));
	  break;
	case hitambig_hack:
	  //	  _ambigresolver.push_back(new PocaAmbigResolver(pset));
	  _ambigresolver.push_back(new HitAmbigResolverHack(pset,_herr[iambig]));
	  break;
      }
    }


  }

  KalFitHack::~KalFitHack(){
    for(size_t iambig=0;iambig<_ambigresolver.size();++iambig){
      delete _ambigresolver[iambig];
    }
    delete _bfield;

    //    delete fStopwatch;
  }

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
  void KalFitHack::findDoublets (KalRep* krep, std::vector<TrkStrawHit*> *hits, DoubletCollection *dcol){

    //first step: create list of doublets
   //    const TrkHotList* hot_list = krep->hotList();
    mu2e::TrkStrawHit* hit;

    int nhits = hits->size();
    int station,panel;
    int oldStation(-1), oldPanel(-1), idlast(0);
    
    CLHEP::Hep3Vector wdir, pos, posPanel, wpos[10], tmppos;    
    CLHEP::Hep3Vector tdir, trkpos;
    HepPoint          tpos;

    double flen;
    double endTrk = 0.0;//krep->endFoundRange();

    int    trkshsize, shId, layer, istraw;
    double doca, rdrift, rdrift1, rdrift2;
    double phiPanel;

    const Straw   *straw, *tmpstraw;

    if (_debug >0){
      printf("[KalFitHack::findDoublets]-------------------------------------------------\n");
      printf("[KalFitHack::findDoublets]  i  shId  ch  panel  il   iw   driftR       doca\n");
      printf("[KalFitHack::findDoublets]-------------------------------------------------\n");
    }
    for (int i=0; i< nhits; ++i){
      hit                    = hits->at(i);//(const mu2e::TrkStrawHit*) &(*it);
      Straw const* straw     = &hit ->straw();
      wdir                   = straw->getDirection();
      pos                    = straw->getMidPoint();
      station                = straw->id().getDevice();
      panel                  = straw->id().getSector();
      shId                   = straw->index().asInt();

      //track info 
      HelixTraj trkHel(krep->helix(endTrk).params(),krep->helix(endTrk).covariance());
      flen    = trkHel.zFlight(pos.z());
      krep->traj().getInfo(flen, tpos, tdir);
      trkpos  = CLHEP::Hep3Vector(tpos.x(), tpos.y(), tpos.z());

      //calculate distance of closest approach - from midwire to track
      HepPoint p1(pos.x(),pos.y(),pos.z());
      HepPoint p2(trkpos.x() ,trkpos.y() ,trkpos.z());
      
      TrkLineTraj trstraw(p1, wdir, 0., 0.);
      TrkLineTraj trtrk  (p2, tdir, 0., 0.);
//-----------------------------------------------------------------------------
// distance of closest approach calculated from the track to the hit,
// track trajectory is the first parameter
//-----------------------------------------------------------------------------
      //      TrkPoca poca  (trstraw, 0.,trtrk   , 0.);
      TrkPoca poca  (trtrk,0.,trstraw,0.);
      doca   = poca.doca();
      
      //get the drift radius
      rdrift  =  hit->driftRadius();

      if (_debug >0){
	layer  = straw->id().getLayer();
	istraw = straw->id().getStraw();
	printf("[KalFitHack::findDoublets] %2i  %5i %3i  %4i  %3i %3i %8.3f %8.3f\n",
	       i, shId, station, panel, layer, istraw, rdrift, doca);
      }
//-----------------------------------------------------------------------------
// do not use straw hits with small drift radius
//-----------------------------------------------------------------------------
      if (rdrift < fMinHitDrift)                           goto NEXT_STRAW_HIT;

      if (station != oldStation) { 
	dcol->push_back(Doublet(station, panel, wdir, tdir, trkpos, hit));
	oldStation = station;
	oldPanel   = panel;
	++idlast;
      } 
      else {
	if (panel == oldPanel) {
	  dcol->at(idlast-1).fTrkshcol.push_back(hit);
	  dcol->at(idlast-1).fTrkDirCol.push_back(tdir);
	  dcol->at(idlast-1).fTrkPosCol.push_back(trkpos);
	}
	else {
	  dcol->push_back(Doublet(station, panel, wdir, tdir, trkpos, hit));
	  oldStation = station;
	  oldPanel   = panel;
	  ++idlast;
	}
      }
	
    NEXT_STRAW_HIT:;
    }
//-----------------------------------------------------------------------------
// resolve ambiguities for doublets
//-----------------------------------------------------------------------------
    int      dcolsize  = dcol->size();
    double   lineSlopes[4];
    int  ambStrawA[4], ambStrawB[4];
    for (int i=0; i<4; ++i){
      lineSlopes[i] = 0;
      ambStrawA [i] = 0;
      ambStrawB [i] = 0;
    }

    Doublet *doub;
    if (_debug >0){
      printf("[KalFitHack::findDoublets] BEGIN iherr:%i \n", fAnnealingStep);
      printf("[KalFitHack::findDoublets]-----------------------------------------------------------------------------------------------------------------------------------------\n");
      printf("[KalFitHack::findDoublets]  i  shId ch pnl lay str      x        y         z      sinphi  tksphi    xtrk     ytrk      ztrk      xr      yr     zr      doca   rdr \n");
      printf("[KalFitHack::findDoublets]-----------------------------------------------------------------------------------------------------------------------------------------\n");
    }

    for (int i=0; i<dcolsize; ++i){
      doub      = &dcol->at(i);
      trkshsize = doub->fTrkshcol.size();
    
      for (int j=0; j<trkshsize; ++j){
	hit   = doub->fTrkshcol.at(j);
	straw = &hit->straw();
	shId  = straw->index().asInt();

	if (j==0){
//-----------------------------------------------------------------------------
// assume wires are perpendicular to the radial direction to the panel
//-----------------------------------------------------------------------------
	  posPanel = straw->getMidPoint();
	  phiPanel = std::atan(posPanel.y()/posPanel.x());
	  posPanel = CLHEP::HepRotationZ(-phiPanel)*posPanel;
	}

	// mid-wire position and the wire direction
	wpos[j] = straw->getMidPoint();
	wdir    = doub->fShDir;
	
	//get track info
	trkpos  = doub->fTrkPosCol.at(j);
	tdir    = doub->fTrkDirCol.at(j);

	HepPoint p1(wpos[j].x(),wpos[j].y(),wpos[j].z());
	HepPoint p2(trkpos.x() ,trkpos.y() ,trkpos.z());
	
	TrkLineTraj trstraw(p1, wdir, 0., 0.);
	TrkLineTraj trtrk  (p2, tdir, 0., 0.);
	TrkPoca     poca   (trstraw, 0.,trtrk   , 0.);
	
	doca   = poca.doca();
	rdrift = hit->driftRadius();

	// rotate wire positions
	wpos[j] = CLHEP::HepRotationZ(-phiPanel)*wpos[j];
	trkpos  = CLHEP::HepRotationZ(-phiPanel)*trkpos;
	tdir    = CLHEP::HepRotationZ(-phiPanel)*tdir;

	if (trkshsize == 2) {
	  hit  = doub->fTrkshcol.at(0);
	  tmpstraw  = &hit->straw();
	  pos     = tmpstraw->getMidPoint();;
	  pos     = CLHEP::HepRotationZ(-phiPanel)*pos;
	  rdrift1 = hit->driftRadius();
	  
	  hit      = doub->fTrkshcol.at(1);
	  tmpstraw = &hit->straw();
	  tmppos   = straw->getMidPoint();;
	  tmppos   = CLHEP::HepRotationZ(-phiPanel)*tmppos;
	  rdrift2  = hit->driftRadius();
//-----------------------------------------------------------------------------
// assume to be in a local coordinate system of the panel
// pos : 1st wire, tmppos - 2nd wire
//-----------------------------------------------------------------------------
	  findLines(pos.z()   , pos.x()   , rdrift1,
		    tmppos.z(), tmppos.x(), rdrift2,
		    lineSlopes, ambStrawA, ambStrawB);
	}
	
	if (_debug > 0) {
	  printf("[KalFitHack::findDoublets] %2i %5i %2i %3i %3i %3i %9.3f %9.3f %9.3f  %6.3f  %6.3f %9.3f %8.3f %9.3f %8.3f %4.1f %9.3f %6.3f %5.3f\n",
		 i, shId, doub->fStationId, doub->fPanelId, 
		 straw->id().getLayer(),
		 straw->id().getStraw(),
		 straw->getMidPoint().x(), straw->getMidPoint().y(), straw->getMidPoint().z(),
		 wdir.y(), 
		 tdir.x()/tdir.z(),
		 trkpos.x(), trkpos.y(), trkpos.z(),
		 wpos[j].x(), wpos[j].y(), wpos[j].z(),
		 doca,
		 rdrift);
	}
      }
    }
  }

//-----------------------------------------------------------------------------
// first step: create list of doublets
//-----------------------------------------------------------------------------
  void KalFitHack::markDoublets (DoubletCollection *dcol){

    double kMaxSlopeDist(0.10);

    mu2e::TrkStrawHit* hit;
   
    CLHEP::Hep3Vector wdir, pos,  wpos[10], posPanel, tmppos;    
    CLHEP::Hep3Vector tdir, trkpos, trkpos1, trkpos2, tdir1, tdir2;
    CLHEP::Hep3Vector wdir1, wdir2;
    HepPoint          tpos;

    int               trkshsize;
    double            doca1, doca2, rdrift1, rdrift2, phiPanel, dsl;
    Doublet           *doub;
    
    int               shId1, shId2, trkSlopeSign, nsol;

    double            lineSlopes[4] = {0, 0, 0, 0};
    int               ambStrawA [4] = {0, 0, 0, 0};
    int               ambStrawB [4] = {0, 0, 0, 0};
    double            slopeDist, trkslope, trkslope1, trkslope2;

    const Straw       *tmpstraw, *tmpstraw2;
    
    if (_debug >0){
      printf("[KalFitHack::markDoublets] BEGIN iherr:%i \n", fAnnealingStep);
      printf("[KalFitHack::markDoublets]---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
      printf("[KalFitHack::markDoublets]  i  shId ch pnl il is      x       y        z       cth    trkth    xtrk      ytrk       ztrk     xr     yr      zr     doca    rdr   am     s1  amb1    s2  amb1   s3   amb3   s4   amb4 ns\n");
      printf("[KalFitHack::markDoublets]---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    }
    
    int ndoublets  = dcol->size();

    for (int i=0; i<ndoublets; ++i){
      doub      = &dcol->at(i);
      trkshsize = doub->fTrkshcol.size();
      
      if (trkshsize < 2) continue; // goto next doublet

      wdir = doub->fShDir;

      if (trkshsize == 2) {

	for (int j=0; j<4; ++j){
	  lineSlopes[j] = 0;
	  ambStrawB [j] = 0;
	  ambStrawA [j] = 0;
	}

	hit       = doub->fTrkshcol.at(0);
	tmpstraw  = &hit->straw();

	posPanel  = tmpstraw->getMidPoint();
	phiPanel  = std::atan2(posPanel.y(),posPanel.x());
	posPanel  = CLHEP::HepRotationZ(-phiPanel)*posPanel;

	pos       = tmpstraw->getMidPoint();;
	pos       = CLHEP::HepRotationZ(-phiPanel)*pos;
	rdrift1   = hit->driftRadius();
	shId1     = tmpstraw->index().asInt();
	  
	hit       = doub->fTrkshcol.at(1);
	tmpstraw2 = &hit->straw();
	tmppos    = tmpstraw2->getMidPoint();;
	tmppos    = CLHEP::HepRotationZ(-phiPanel)*tmppos;
	rdrift2   = hit->driftRadius();
	shId2     = tmpstraw2->index().asInt();

	tdir1     = doub->fTrkDirCol.at(0);
	tdir1     = CLHEP::HepRotationZ(-phiPanel)*tdir1;
	trkslope1 = tdir1.x()/tdir1.z();

	tdir2     = doub->fTrkDirCol.at(1);
	tdir2     = CLHEP::HepRotationZ(-phiPanel)*tdir2;
	trkslope2 = tdir2.x()/tdir2.z();
//-----------------------------------------------------------------------------
// for the track dx/dz use average of the two dx/dz slopes calculated 
// in the two layers corresponding to the doublet hits
//-----------------------------------------------------------------------------
	trkslope  = (trkslope1 + trkslope2)/2.;

	trkSlopeSign = int(-1.*trkslope/std::fabs(trkslope));

	findLines(pos.z()   , pos.x()   , rdrift1,
		  tmppos.z(), tmppos.x(), rdrift2,
		  lineSlopes, ambStrawA, ambStrawB);
	  
	nsol      = 0;
	slopeDist = kMaxSlopeDist;

	for (int i=0; i<4; i++) {
	  dsl = fabs(trkslope - lineSlopes[i]);
	  if (dsl <  kMaxSlopeDist) {
	    if (dsl <  slopeDist) {
	      fAmbigVecSlope[shId1] = trkSlopeSign*ambStrawA[i];
	      fAmbigVecSlope[shId2] = trkSlopeSign*ambStrawB[i];
	      slopeDist             = dsl;
	    }
	    nsol             += 1;
	  }
	}

// 	if (std::fabs(trkslope - lineSlopes[0]) < maxSlopeDist) {
// 	  if( std::fabs(trkslope - lineSlopes[0]) < slopeDist){
// 	    fAmbigVecSlope[shId1] = trkSlopeSign*ambStrawA[0];
// 	    fAmbigVecSlope[shId2] = trkSlopeSign*ambStrawB[0];
// 	    slopeDist         = fabs(trkslope - lineSlopes[0]);
// 	  }
// 	  nsol             += 1;
// 	}

// 	if (std::fabs(trkslope - lineSlopes[1]) < maxSlopeDist) {
// 	  if (std::fabs(trkslope - lineSlopes[1]) < slopeDist){
// 	    fAmbigVecSlope[shId1] = trkSlopeSign*ambStrawA[1];
// 	    fAmbigVecSlope[shId2] = trkSlopeSign*ambStrawB[1];
// 	    slopeDist        = std::fabs(trkslope - lineSlopes[1]);
// 	  }
// 	  nsol             += 1;
// 	}

// 	if (std::fabs(trkslope - lineSlopes[2]) < maxSlopeDist) {
// 	  if (std::fabs(trkslope - lineSlopes[2]) < slopeDist){
// 	    fAmbigVecSlope[shId1] = trkSlopeSign*ambStrawA[2];
// 	    fAmbigVecSlope[shId2] = trkSlopeSign*ambStrawB[2];
// 	    slopeDist        = std::fabs(trkslope - lineSlopes[2]);
// 	  }
// 	  nsol             += 1;
// 	}
	  	  
// 	if (std::fabs(trkslope - lineSlopes[3]) < maxSlopeDist) {
// 	  if (std::fabs(trkslope - lineSlopes[3]) < slopeDist){
// 	    fAmbigVecSlope[shId1] = trkSlopeSign*ambStrawA[3];
// 	    fAmbigVecSlope[shId2] = trkSlopeSign*ambStrawB[3];
// 	    slopeDist        = std::fabs(trkslope - lineSlopes[3]);
// 	  }
// 	  nsol             += 1;
// 	}

	double minDriftRadius = fMinDriftDoublet;
	if (fAnnealingStep >0) {
	  minDriftRadius  = fMinDriftDoublet/2.;
	}
//-----------------------------------------------------------------------------
// when the two hits have the same drift sign and close drift times,
// the two (++) and (--) solutions may be difficult to distinguish
// require the solution to be unique within the window defined by kMaxSlopeDist
//-----------------------------------------------------------------------------
	if (nsol ==1 ){
	  if ((rdrift2 > minDriftRadius) && (rdrift1 > minDriftRadius)) {
	    if ((fAmbigVecSlope[shId1] + fAmbigVecSlope[shId2] == 0)) {
	      fAmbigVec[shId1] = fAmbigVecSlope[shId1];
	      fAmbigVec[shId2] = fAmbigVecSlope[shId2];
	    } else if (std::fabs(rdrift1-rdrift2) > fDeltaDriftDoublet){
	      fAmbigVec[shId1] = fAmbigVecSlope[shId1];
	      fAmbigVec[shId2] = fAmbigVecSlope[shId2];
	    }
	  }
	}

	if (_debug >0){
	  trkpos1  = doub->fTrkPosCol.at(0);
	  trkpos2  = doub->fTrkPosCol.at(1);
	  
	  tdir1    = doub->fTrkDirCol.at(0);
	  tdir2    = doub->fTrkDirCol.at(1);
	  
	  HepPoint    p1(tmpstraw->getMidPoint().x(),tmpstraw->getMidPoint().y(),tmpstraw->getMidPoint().z());
	  HepPoint    p2(trkpos1.x() ,trkpos1.y() ,trkpos1.z());

	  TrkLineTraj trstraw(p1, tmpstraw->getDirection(), 0., 0.);
	  TrkLineTraj trtrk  (p2, tdir1, 0., 0.);

	  TrkPoca     poca1  (trstraw, 0.,trtrk   , 0.);
	  
	  doca1   = poca1.doca();

	  p1 = HepPoint(tmpstraw2->getMidPoint().x(),tmpstraw2->getMidPoint().y(),tmpstraw2->getMidPoint().z());
	  p2 = HepPoint(trkpos2.x() ,trkpos2.y() ,trkpos2.z());

	  trstraw = TrkLineTraj(p1, tmpstraw2->getDirection(), 0., 0.);
	  trtrk   = TrkLineTraj(p2, tdir2, 0., 0.);
	  TrkPoca poca2  (trstraw, 0.,trtrk   , 0.);
	  
	  doca2   = poca2.doca();

	  printf("[KalFitHack::markDoublets] %2i %5i %2i %3i %2i %2i %8.3f %8.3f %9.3f %6.3f  %6.3f  %8.3f  %8.3f %9.3f %5.3f %4.1f %9.3f %6.3f %6.3f ",
		 i, shId1, doub->fStationId, doub->fPanelId, 
		 tmpstraw->id().getLayer(),
		 tmpstraw->id().getStraw(),
		 tmpstraw->getMidPoint().x(), tmpstraw->getMidPoint().y(), tmpstraw->getMidPoint().z(),
		 wdir.y(), 
		 trkslope,
		 trkpos1.x(), trkpos1.y(), trkpos1.z(),
		 pos.x(), pos.y(), pos.z(),
		 doca1,
		 rdrift1); 

	  printf("%5i %6.3f %2i%2i %6.3f %2i%2i %6.3f %2i%2i %6.3f %2i%2i %2i\n",
		 fAmbigVec[shId1],
		 lineSlopes[0], trkSlopeSign*ambStrawA[0], trkSlopeSign*ambStrawB[0],
		 lineSlopes[1], trkSlopeSign*ambStrawA[1], trkSlopeSign*ambStrawB[1],
		 lineSlopes[2], trkSlopeSign*ambStrawA[2], trkSlopeSign*ambStrawB[2],
		 lineSlopes[3], trkSlopeSign*ambStrawA[3], trkSlopeSign*ambStrawB[3], 
		 nsol);
	
	  printf("[KalFitHack::markDoublets] %2i %5i %2i %3i %2i %2i %8.3f %8.3f %9.3f %6.3f  %6.3f  %8.3f  %8.3f %9.3f %5.3f %4.1f %9.3f %6.3f %6.3f ",
		 i, shId2, doub->fStationId, doub->fPanelId,  
		 tmpstraw2->id().getLayer(),
		 tmpstraw2->id().getStraw(),
		 tmpstraw2->getMidPoint().x(), tmpstraw2->getMidPoint().y(), tmpstraw2->getMidPoint().z(),
		 wdir.y(), 
		 trkslope,
		 trkpos2.x(), trkpos2.y(), trkpos2.z(),
		 tmppos.x(), tmppos.y(), tmppos.z(),
		 doca2,
		 rdrift2); 
	  printf("%5i %6.3f %2i%2i %6.3f %2i%2i %6.3f %2i%2i %6.3f %2i%2i %2i\n",
		 fAmbigVec[shId2],
		 lineSlopes[0], trkSlopeSign*ambStrawA[0], trkSlopeSign*ambStrawB[0],
		 lineSlopes[1], trkSlopeSign*ambStrawA[1], trkSlopeSign*ambStrawB[1],
		 lineSlopes[2], trkSlopeSign*ambStrawA[2], trkSlopeSign*ambStrawB[2],
		 lineSlopes[3], trkSlopeSign*ambStrawA[3], trkSlopeSign*ambStrawB[3],
		 nsol);
	}
      }
    }
  }

  //--------------------------------------------------------------------------------
  void KalFitHack::findAndMarkMultiplets(KalRep* krep,   std::vector<TrkStrawHit*> *hits){
    for (int i=0; i<40000; ++i)	{
      fAmbigVec[i]      = -9999;			
      fAmbigVecSlope[i] = -9999;
    }	
    DoubletCollection dcol;
    //create the collection of doublets
    findDoublets (krep, hits, &dcol);
    
    if (dcol.size() > 0 ){
    //fix the ambiguity of the doublets and 
    //store indeces of these doublets
    markDoublets(&dcol);
    } else {
      if (_debug >0) {
	printf("[KalFitHack::findAndMarkMultiplets] no multiplets found!\n");
      }
    }

  }


//-----------------------------------------------------------------------------
  void KalFitHack::setMultipletsAmbig(std::vector<TrkStrawHit*> *hits){
    int          nhits = hits->size();
    int          hitIndex;
    TrkStrawHit* hit;

    if (_debug > 0){
      printf("-----------------------------------------------------------------\n");
      printf("[KalFitHack::setMultipletsAmbig]   shId  |  sec  | panel | ambig |\n");
      printf("-----------------------------------------------------------------\n");
    }
    for (int i=0; i<nhits; ++i){
      hit      = hits->at(i);
      hitIndex = hit->index();
      Straw const* straw = &hit->straw();
      int          shId  = straw->index().asInt();

      if (fAmbigVec[shId] > -10){
	hit->setAmbig(fAmbigVec[shId]);
	hit->setAmbigUpdate(false);
	if (_debug > 0){
	  Straw const* straw    = &hit->straw();
	  int          shId     = straw->index().asInt();
	  int          station  = straw->id().getDevice();
	  int          panel    = straw->id().getSector();

	  printf("[KalFitHack::setMultipletsAmbig]  %5i | %3i | %3i | %3i | %3i |\n", shId, station, panel, hitIndex, fAmbigVec[shId]);
	}
      }
    }
  }

//-----------------------------------------------------------------------------------------
//2015 - 02 -20 G. Pezzu addedd the function for calculataing the slope of the lines
// tangent to two given circles
//-----------------------------------------------------------------------------------------

//Then the tangent lines, expressed as (costheta)x+(sintheta)y+C=0, 
//and parameterized by theta and C are the four solutions of:
// |xA cos theta + yA sin theta + C| = rA
// and
// |xB cos theta + yB sin theta + C| = rB
//Solving these equations for costheta and sintheta isn't easy, but when we do, we get:
// A = costheta = ((xA-xB)(±rA±rB) + (yA-yB) sqrt((xA-xB)2+(yA-yB)2-(±rA±rB)2))/((xA-xB)2+(yA-yB)2)
// B = sintheta = ((yA-yB)(±rA±rB) - (xA-xB) sqrt((xA-xB)2+(yA-yB)2-(±rA±rB)2))/((xA-xB)2+(yA-yB)2)

// points tangent to the circles are
//(xA±-rA cos theta, yA±-rA sin theta) and
//(xB±rB cos theta , yB±rB sin theta) 


  void KalFitHack::findLines(double xa, double ya, double ra,
			     double xb, double yb, double rb,
			     double *slopes, int *ambStrawA, int *ambStrawB){
    
    double            cosVec[4], sinVec[4], dx, dy, dr2;

    CLHEP::Hep3Vector dir1;               // traj direction
    CLHEP::Hep3Vector dir2(0., 0., -1.);  // wire direction

    CLHEP::Hep3Vector pos1, pos2, delta, between;
    double signA[4] = {+1, +1, -1, -1};
    double signB[4] = {+1, -1, -1, +1};

    dx  = xa-xb;
    dy  = ya-yb;
    dr2 = dx*dx+dy*dy;

    cosVec[0] = (dx*(+ra+rb) + dy*sqrt(dr2 - (+ra+rb)*(+ra+rb)))/dr2;
    cosVec[1] = (dx*(+ra-rb) + dy*sqrt(dr2 - (+ra-rb)*(+ra-rb)))/dr2;
    cosVec[2] = (dx*(-ra-rb) + dy*sqrt(dr2 - (-ra-rb)*(-ra-rb)))/dr2;
    cosVec[3] = (dx*(-ra+rb) + dy*sqrt(dr2 - (-ra+rb)*(-ra+rb)))/dr2;
    
    sinVec[0] = (dy*(+ra+rb) - dx*sqrt(dr2 - (+ra+rb)*(+ra+rb)))/dr2;
    sinVec[1] = (dy*(+ra-rb) - dx*sqrt(dr2 - (+ra-rb)*(+ra-rb)))/dr2;
    sinVec[2] = (dy*(-ra-rb) - dx*sqrt(dr2 - (-ra-rb)*(-ra-rb)))/dr2;
    sinVec[3] = (dy*(-ra+rb) - dx*sqrt(dr2 - (-ra+rb)*(-ra+rb)))/dr2;
    
//     slopes[0] = -cosVec[0]/sinVec[0]; // a->+1, b->-1
//     slopes[1] = -cosVec[1]/sinVec[1]; // a->+1, b->+1
//     slopes[2] = -cosVec[2]/sinVec[2]; // a->-1, b->+1
//     slopes[3] = -cosVec[3]/sinVec[3]; // a->-1, b->-1
    
    // now determine the ambiguities

    for (int i=0; i<4; ++i){

      slopes[i] = -cosVec[i]/sinVec[i];

      dir1.setX(-cosVec[i]);
      dir1.setY( sinVec[i]);

      pos1.set(xa-ra*signA[i]*cosVec[i],ya-ra*signA[i]*sinVec[i],0);
      pos2.set(xa,ya,0);

      delta   = pos2 - pos1;
      between = dir1.cross(dir2).unit();
    
      ambStrawA[i] = delta.dot(between) > 0.0 ? 1 : -1;
  
      pos1.set(xb+rb*signB[i]*cosVec[i],yb+rb*signB[i]*sinVec[i],0);
      pos2.set(xb, yb,0);

      delta   = pos2 - pos1;
   
      ambStrawB[i] = delta.dot(between) > 0.0 ? 1 : -1;
    }
  }

//------------------------------------------------------------------------------------------
  void KalFitHack::makeTrack(KalFitResult& kres, CalTimePeak* TPeak, int markDoubs) {

    kres._fit = TrkErrCode(TrkErrCode::fail);

					// test if fitable
    if (fitable(kres._tdef)) {
					// first, find t0
      TrkT0 t0;
      bool caloInitCond(false);
      if (TPeak->Cluster() != NULL) {
	caloInitCond = true;
      }

      if (_initt0) {
	if (!caloInitCond) {
	  initT0(kres._tdef, t0);
	} 
	else {
	  initCaloT0(TPeak, kres._tdef, t0);
	}
      }
      else {
	t0 = kres._tdef.t0();
      }
					// knowing t0, create the hits
      makeHits(kres, t0);
//-----------------------------------------------------------------------------
// Create the BaBar hit list, and fill it with these hits.  The BaBar list takes ownership
// This will go away when we cleanup the BaBar hit storage, FIXME!!!
//-----------------------------------------------------------------------------
      TrkHotListFull* hotlist = new TrkHotListFull();
      for(std::vector<TrkStrawHit*>::iterator ihit=kres._hits.begin();ihit!=kres._hits.end();ihit++){
        TrkStrawHit* trkhit = *ihit;
	hotlist->append(trkhit);
      }
//-----------------------------------------------------------------------------
// Find the wall and gas material description objects for these hits
//-----------------------------------------------------------------------------
      if (_matcorr) makeMaterials(kres);
// create Kalman rep
      kres._krep = new KalRep(kres._tdef.helix(), hotlist, kres._detinter, *this, kres._tdef.particle());
      assert(kres._krep != 0);
// initialize krep t0; eventually, this should be in the constructor, FIXME!!!
      double flt0 = kres._tdef.helix().zFlight(0.0);
      kres._krep->setT0(t0,flt0);

      if (_debug>0){
	printHits(kres);
      }
//-----------------------------------------------------------------------------
// now fit
// 10-07-2013 giani added the following line. It updates the hit times
//            following changes in the t0 value 
//-----------------------------------------------------------------------------
      if(caloInitCond) updateHitTimes(kres);

      fMarkDoublets = markDoubs;
//-----------------------------------------------------------------------------
// 09 - 26 - 2013 giani 
// include the calorimeter information when it is avaiable
//-----------------------------------------------------------------------------
      if ((_daveMode == 0) && caloInitCond) {
	fitTrack(kres, TPeak);
      } 
      else {
	fitTrack(kres);
      }

      if (_removefailed) kres.removeFailed();
    }
  }

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
  void KalFitHack::addHits(KalFitResult&              kres   , 
			   const StrawHitCollection*  straws , 
			   std::vector<hitIndex>      indices, 
			   double                     maxchi ) {

					// there must be a valid Kalman fit to add hits to

    if(kres._krep != 0 && kres._fit.success()){
      ConditionsHandle<TrackerCalibrations> tcal("ignored");
      const Tracker& tracker = getTrackerOrThrow();
      std::vector<TrkStrawHit*>::iterator ihigh;
      std::vector<TrkStrawHit*>::reverse_iterator ilow;
//-----------------------------------------------------------------------------
// use the reference trajectory, as that's what all the existing hits do
//-----------------------------------------------------------------------------
      const TrkDifPieceTraj* reftraj = kres._krep->referenceTraj();

      if (_debug>0){
	printf("[KalFitHack::addHits]  shId  |  sec | panel |  res  | drift |  chi  |\n");
      }

      for(unsigned iind=0;iind<indices.size(); ++iind){
	size_t istraw = indices[iind]._index;
	const StrawHit& strawhit(straws->at(istraw));
	const Straw& straw = tracker.getStraw(strawhit.strawIndex());
// estimate  initial flightlength
	double hflt(0);
	TrkHelixUtils::findZFltlen(*reftraj,straw.getMidPoint().z(),hflt);
// find the bounding sites near this hit, and extrapolate to get the hit t0
	std::sort(kres._hits.begin(),kres._hits.end(),fltlencomp(kres._tdef.fitdir().fitDirection()));
	findBoundingHits(kres._hits,hflt,ilow,ihigh);
	const TrkStrawHit* nearhit;
	if(ihigh != kres._hits.end())
	  nearhit = *ihigh;
	else
	  nearhit = *ilow;
	TrkT0 hitt0 = nearhit->hitT0();
	double mom = kres._krep->momentum(nearhit->fltLen()).mag();
	double beta = kres._tdef.particle().beta(mom);
	double tflt = (hflt-nearhit->fltLen())/(beta*CLHEP::c_light);
// update the time in the TrkT0 object
	hitt0._t0 += tflt;
// create the hit object.  Assume we're at the last iteration over added error
	TrkStrawHit* trkhit = new TrkStrawHit(strawhit,straw,istraw,hitt0,hflt,_herr.back(),_maxdriftpull);
	assert(trkhit != 0);
// allow the hit to update its own ambiguity for now: eventually we should get the resolver to do this, FIXME!!!
	trkhit->setAmbigUpdate(true);
// must be initialy active for KalRep to process correctly
	trkhit->setActivity(true);
// flag the added hit
	trkhit->setUsability(3);
// add the hit to the track and the fit
	kres._krep->addHot(trkhit);
	kres._hits.push_back(trkhit);
// create intersections for the material of this hit and add those to the track
	DetIntersection wallinter;
	if(trkhit->wallElem().reIntersect(reftraj,wallinter))
	  kres._krep->addInter(wallinter);	
	DetIntersection gasinter;
	if(trkhit->gasElem().reIntersect(reftraj,gasinter))
	  kres._krep->addInter(gasinter);
// check the raw residual: This call works because the HOT isn't yet processed as part of the fit.
        double chi = fabs(trkhit->residual()/trkhit->hitRms());
	if (_debug>0){
	  printf("[KalFitHack::addHits] %5i | %3i | %3i | %10.3f | %10.3f | %10.3f |\n",
		 straw.index().asInt(),
		 straw.id().getDevice(),
		 straw.id().getSector(),
		 trkhit->residual(),
		 trkhit->driftRadius(),
		 chi);
	}
// if it's outside limits, deactivate the HOT
	if (chi > maxchi || !trkhit->physicalDrift(maxchi)) {
	  trkhit->setActivity(false);
	}
// now that we've got the residual, turn off auto-ambiguity resolution
	trkhit->setAmbigUpdate(false);
      }
 // sort hits by flightlength (or in Z, which is the same)
      std::sort(kres._hits.begin(),kres._hits.end(),fltlencomp(kres._tdef.fitdir().fitDirection()));

// refit the track one more time with minimal external errors
      fitIteration(kres,_herr.size()-1);
      kres._krep->addHistory(kres._fit,"AddHits");
    }
  }


  void KalFitHack::fitTrack(KalFitResult& kres, CalTimePeak* TPeak) {
// loop over external hit errors, ambiguity assignment, t0 toleratnce
// 10-03-2013 giani changed this loop. now it loops on all the stations
// and store the last fit that converges

    for (size_t iherr=0;iherr<_herr.size();++iherr){
      //      condition = false;
      if(TPeak){
	fitIteration(kres,iherr,TPeak);
      } else{
	fitIteration(kres,iherr);
      }
      if (! kres._fit.success()) break; //commented by gianipez
      // if(iherr==0){
// 	condition = false;
// 	chisqN = kres._krep->chisq();
// 	fitIndex = 0;
//       }else {
// 	condition = (kres._krep->chisq() < chisqN);
//       }
//       if( kres._fit.success() && condition ) {
// 	fitIndex = int(iherr);
// 	chisqN   = kres._krep->chisq();
//       }
    }

    if(kres._krep != 0) kres._krep->addHistory(kres._fit,"KalFitHack");
  }

//-----------------------------------------------------------------------------
// one step of the track fit
//-----------------------------------------------------------------------------
  void KalFitHack::fitIteration(KalFitResult& kres, size_t iherr, CalTimePeak* TPeak) {
    // update external hit errors.  This isn't strictly necessary on the 1st iteration.

    if (_debug >0) {
      printf("------------------------------------------------------------------------------------------\n");
      printf("[KalFitHack::fitIteration] BEGIN iherr:%i \n", int(iherr));
      printf("------------------------------------------------------------------------------------------\n");
    }
//     for(std::vector<TrkStrawHit*>::iterator itsh = kres._hits.begin(); itsh != kres._hits.end(); ++itsh){
//       (*itsh)->setExtErr(_herr[iherr]);
//     }

//-----------------------------------------------------------------------------------
// 2015 -02 -17 G. Pezzullo: loop over the hits and assign a smaller external error 
// for the doublets
//-----------------------------------------------------------------------------------
    fAnnealingStep = iherr;
    if ( (int(iherr)<fILoopMarkDoublets) && (fMarkDoublets==1)){
      
      //--------------------------------------------------------------------------------
      //2015 - 02 - 19 G. Pezzu added the following function
      // for re-searching multiplets using updated fit results
      findAndMarkMultiplets(kres._krep, &kres._hits);
      
      setMultipletsAmbig(&kres._hits);
      if (_debug>0){
	//	printf("[KalFitHack::fitIteration] BEGIN iherr:%i \n", int(iherr));
	printHits(kres);
      }
    }

    TrkStrawHit* hit;
    int          shId, nhits;

    nhits = kres._hits.size();
    for (int i=0; i<nhits; ++i){
      hit   = kres._hits.at(i);
      shId  = hit->straw().index().asInt();

      if (int(iherr) < fILoopMarkDoublets){
	if (fAmbigVec[shId] < -10){
	  hit->setExtErr(_herr[iherr]);
	}else {
	  hit->setExtErr(_herr[iherr]/fScaleErrDoublet);
	}
      }else {
	hit->setExtErr(_herr[iherr]);
      }
    }

    // update t0, and propagate it to the hits
    double   oldt0 = kres._krep->t0()._t0;
    unsigned niter(0);
    bool     changed(true);
    bool     fit_success;

    kres._nt0iter = 0;
    kres._fit     = TrkErrCode::succeed;

    while(kres._fit.success() && changed && niter < maxIterations()) {
      //      printf("[KalFitHack::fitIteration] new iteration: iherr:%i niter:%i\n", iherr,niter);
      changed = false;
      _ambigresolver[iherr]->resolveTrk(kres);
//--------------------------------------------------------------------------------
//2015-02-17 G. Pezzu: fix the ambiguity of the doublets!
//2015-02-19 G. Pezzu: re-search hit multiplets using updated fit results
//--------------------------------------------------------------------------------
      if ( (int(iherr) < fILoopMarkDoublets) && (fMarkDoublets == 1)) {
	findAndMarkMultiplets(kres._krep, &kres._hits);
	
	setMultipletsAmbig(&kres._hits);
	if (_debug>0){
	  printHits(kres);
	}
      }
//--------------------------------------------------------------------------------
      kres._krep->resetFit();
      kres.fit();

      fit_success = kres._fit.success();
      if(! fit_success) break;
      if(_updatet0 ){
	// 2014-12-11: G.Pezzullo and P.Murat - temporary *FIXME*
	if (TPeak != NULL){
	  updateCalT0(kres,TPeak);
	} else{
	  updateT0(kres);
	}
	changed |= fabs(kres._krep->t0()._t0-oldt0) > _t0tol[iherr];
	oldt0 = kres._krep->t0()._t0;
      }
      // drop outlyers
      if(_weedhits){
	kres._nweediter = 0;
	changed |= weedHits(kres);
      }
      niter++;
    }
//----------------------------------------------------------------------
// 2014-12-29 G. Pezzullo and P. Murat added the following counter 
    fNIter += niter;

    fit_success = kres._fit.success();
    if (fit_success) {
      _ambigresolver[iherr]->resolveTrk(kres);
//--------------------------------------------------------------------------------
//2015 -02 -17 G. Pezzu: fix the ambiguity of the doublets!
//--------------------------------------------------------------------------------
      if ( (int(iherr) < fILoopMarkDoublets) && (fMarkDoublets==1)){

	findAndMarkMultiplets(kres._krep, &kres._hits);

	setMultipletsAmbig(&kres._hits);
	if (_debug>0){
	  printHits(kres);
	}

      }
    }
    kres._ninter = kres._krep->intersections();

    //    printf("[KalFitHack::fitIteration] END iherr:%i \n",iherr);
  }


//-----------------------------------------------------------------------------
  bool KalFitHack::fitable(TrkDef const& tdef){
    return tdef.strawHitIndices().size() >= _minnstraws;
  }
  
//-----------------------------------------------------------------------------
  void KalFitHack::makeHits(KalFitResult& kres,TrkT0 const& t0) {
    const Tracker& tracker = getTrackerOrThrow();
    TrkDef const& tdef = kres._tdef;
// compute the propagaion velocity
    double flt0 = tdef.helix().zFlight(0.0);
    double mom = TrkMomCalculator::vecMom(tdef.helix(),bField(),flt0).mag();
    double vflt = tdef.particle().beta(mom)*CLHEP::c_light;
    unsigned nind = tdef.strawHitIndices().size();
    for(unsigned iind=0;iind<nind;iind++){
      size_t istraw = tdef.strawHitIndices()[iind]._index;
      const StrawHit& strawhit(tdef.strawHitCollection()->at(istraw));
      const Straw& straw = tracker.getStraw(strawhit.strawIndex());
      double fltlen = tdef.helix().zFlight(straw.getMidPoint().z());
    // estimate arrival time at the wire
      TrkT0 hitt0(t0);
      hitt0._t0 += (fltlen-flt0)/vflt;
    // create the hit object.  Start with the 1st additional error for anealing
      TrkStrawHit* trkhit = new TrkStrawHit(strawhit,straw,istraw,hitt0,fltlen,_herr.front(),_maxdriftpull);
      assert(trkhit != 0);
    // set the initial ambiguity based on the input
      trkhit->setAmbig(tdef.strawHitIndices()[iind]._ambig);
    // refine the flightlength, as otherwise hits in the same plane are at exactly the same flt, which can cause problems
      TrkErrCode pstat = trkhit->updatePoca(&tdef.helix());
      if(pstat.failure()){
        trkhit->setActivity(false);
      }
      kres._hits.push_back(trkhit);
    }
 // sort the hits by flightlength
    std::sort(kres._hits.begin(),kres._hits.end(),fltlencomp(tdef.fitdir().fitDirection()));
  }

  void KalFitHack::printHits(KalFitResult& kres) {
    const KalRep* Trk  = kres._krep;
    double d0(-1.), om(-1.), r(-1.), phi0(-1.), x0(-1.), y0(-1.), chi2N(-1.);

    if (Trk != 0){
      d0    = Trk->helix(0.).d0();
      om    = Trk->helix(0.).omega();
      r     = fabs(1./om);
      phi0  = Trk->helix(0.).phi0();
      x0    =  -(1/om+d0)*sin(phi0);
      y0    =   (1/om+d0)*cos(phi0);
      chi2N = Trk->chisq()/(Trk->nDof());

      printf("[KalFitHack::printHits] BEGIN iherr:%i \n", fAnnealingStep);
      printf("------------------------------------------------------------------------------------------\n");
      printf("  TrkID    Address      Q    momentum       pt       costh       T0     Nact     chi2  N(dof)  FitCons\n");
      printf("------------------------------------------------------------------------------------------\n");

      Hep3Vector trk_mom;
      //      Trk->printAll();
      double h1_fltlen = Trk->firstHit()->kalHit()->hitOnTrack()->fltLen() - 10;
      trk_mom          = Trk->momentum(h1_fltlen);
      double mom       = trk_mom.mag();
      double pt        = trk_mom.perp();
      double costh     = trk_mom.cosTheta();
      double chi2      = Trk->chisq();
      int    ndof      = Trk->nDof ();
      int    nact      = Trk->nActive();
      double t0        = Trk->t0().t0();
      double fit_consistency = Trk->chisqConsistency().consistency();
      int q            = Trk->charge();
      
      printf("%5i   %16p   %2i %10.3f %10.3f %10.3f %10.3f   %3i %10.3f  %3i %10.3e\n",
	     -1,Trk,q,mom,pt,costh,t0,nact,chi2,ndof,fit_consistency);
      
    }
    printf("[KalFitHack::printHits] x0 = %12.5f y0 = %12.5f r = %12.5f chi2 = %12.5g\n",
	   x0, y0, r,  chi2N);

    
    //-----------------------------------------------------------------------------
    // print detailed information about the track hits
    //-----------------------------------------------------------------------------
    //    const TrkHotList* hot_list = Trk->hotList();
    int nhits = kres._hits.size();
    printf("---------------------------------------------------------------------------");
    printf("-------------------------------------------------------------------------------\n");
    printf(" ih U A     len      rms       x          y          z       HitT     HitDt");
    printf("  SInd  Dev Sec Lay  N  Iamb     T0    Rdrift     Xs         Ys          Zs        resid\n");
    printf("---------------------------------------------------------------------------");
    printf("-------------------------------------------------------------------------------\n");

    int i = 0;
    //    for(TrkHotList::hot_iterator it=hot_list->begin(); it<hot_list->end(); it++) {
    for(int it=0; it<nhits; ++it){
      //	  const KalHit* kh = (*it).kalHit();

      // TrkStrawHit inherits from TrkHitOnTrk

      mu2e::TrkStrawHit* hit =  kres._hits.at(it);

      const mu2e::StrawHit* sh = &hit->strawHit();
      mu2e::Straw*   straw = (mu2e::Straw*) &hit->straw();

      double len = hit->fltLen();
      double resid(-9999), rms(-9999);
      HepPoint  plen(-9999,-9999.,-9999);
      if (Trk != 0){
	plen  = Trk->position(len);
	resid =  hit->resid();
	rms   = hit->hitRms();
      }

      printf("%3i %1i %1i %10.3f %6.3f %10.3f %10.3f %10.3f %8.3f %7.3f",
	     ++i,
	     hit->isUsable(),
	     hit->isActive(),
	     len,
	     rms,
	     plen.x(),plen.y(),plen.z(),
	     sh->time(), sh->dt()
	     );

      Hep3Vector pos;
      hit->hitPosition(pos);
      printf("%6i %3i %3i %3i %3i %3i %8.3f %8.3f %10.3f %10.3f %10.3f %10.3f\n",
	     straw->index().asInt(), 
	     straw->id().getDevice(),
	     straw->id().getSector(),
	     straw->id().getLayer(),
	     straw->id().getStraw(),
		 
	     hit->ambig(),
	     hit->hitT0().t0(),
	     hit->driftRadius(),
	     pos.x(),
	     pos.y(),
	     pos.z(),
	     resid
	     );
    }
  }
  

  void
  KalFitHack::makeMaterials(KalFitResult& kres) {
    TrkDef const& tdef = kres._tdef;
    for(std::vector<TrkStrawHit*>::iterator ihit=kres._hits.begin();ihit!=kres._hits.end();ihit++){
      TrkStrawHit* trkhit = *ihit;
      // create wall and gas intersection objects from each straw hit (active or not)
      DetIntersection wallinter;
      wallinter.delem = 0;
      wallinter.pathlen = trkhit->fltLen();
      DetIntersection gasinter;
      gasinter.delem = 0;
      gasinter.pathlen = trkhit->fltLen();
      if(trkhit->wallElem().reIntersect(&tdef.helix(),wallinter))
	kres._detinter.push_back(wallinter);
      if(trkhit->gasElem().reIntersect(&tdef.helix(),gasinter))
	kres._detinter.push_back(gasinter);
    }
  }

  bool
  KalFitHack::weedHits(KalFitResult& kres) {
    // Loop over HoTs and find HoT with largest contribution to chi2.  If this value
    // is greater than some cut value, deactivate that HoT and reFit
    bool retval(false);
    double worst = -1.;
    TrkStrawHit* worstHot = 0;
    for (std::vector<TrkStrawHit*>::iterator iter = kres._hits.begin(); iter != kres._hits.end(); ++iter){
      TrkStrawHit* iHot = *iter;
      if (iHot->isActive()) {
        double resid, residErr;
        if(iHot->resid(resid, residErr, true)){
          double value = fabs(resid/residErr);
          if (value > _maxhitchi && value > worst) {
            worst = value;
            worstHot = iHot;
          }
        }
      }
    }
    if(0 != worstHot){
      retval = true;
      worstHot->setActivity(false);
      worstHot->setUsability(5); // positive usability allows hot to be re-enabled later
      kres.fit();
      kres._krep->addHistory(kres._fit, "HitWeed");
      // Recursively iterate
      kres._nweediter++;
      if (kres._fit.success() && kres._nweediter < _maxweed ) {
        retval |= weedHits(kres);
      }
    }
    return retval;
  }
  
  bool
  KalFitHack::unweedHits(KalFitResult& kres, double maxchi) {
    // Loop over inactive HoTs and find the one with the smallest contribution to chi2.  If this value
    // is less than some cut value, reactivate that HoT and reFit
    bool retval(false);
    double best = 1.e12;
    TrkStrawHit* bestHot = 0;
    for (std::vector<TrkStrawHit*>::iterator iter = kres._hits.begin(); iter != kres._hits.end(); ++iter){
      TrkStrawHit* iHot = *iter;
      if (!iHot->isActive()) {
        double resid, residErr;
        if(iHot->resid(resid, residErr, true)){
          double chival = fabs(resid/residErr);
  // test both for a good chisquared and for the drift radius to be physical
          if (chival < maxchi && iHot->physicalDrift(maxchi) && chival < best) {
            best = chival;
            bestHot = iHot;
          }
        }
      }
    }
    if(0 != bestHot){
      retval = true;
      bestHot->setActivity(true);
      bestHot->setUsability(4);
      kres.fit();
      kres._krep->addHistory(kres._fit, "HitUnWeed");
      // Recursively iterate
      kres._nunweediter++;
      if (kres._fit.success() && kres._nunweediter < _maxweed  ) {
        retval |= unweedHits(kres,maxchi);
      }
    }
    return retval;
  }
  
  BField const&
  KalFitHack::bField() const {
    if(_bfield == 0){
      GeomHandle<BFieldConfig> bfconf;
      if(_fieldcorr){
// create a wrapper around the mu2e field 
	_bfield = new BaBarMu2eField();	
      } else {
// create a fixed field using the nominal value
	GeomHandle<BFieldConfig> bfconf;
	_bfield=new BFieldFixed(bfconf->getDSUniformValue());
	assert(_bfield != 0);
      }
    }
    return *_bfield;
  }
   
  const TrkVolume* 
  KalFitHack::trkVolume(trkDirection trkdir) const {
    //FIXME!!!!
    return 0;
  }

//-----------------------------------------------------------------------------
// time initialization
//-----------------------------------------------------------------------------
  void KalFitHack::initCaloT0(CalTimePeak* TPeak, TrkDef const& tdef, TrkT0& t0) {
//    2014-11-24 gianipez and Pasha removed time offset between caloriemter and tracker

    // get flight distance of z=0
    double t0flt = tdef.helix().zFlight(0.0);
    // estimate the momentum at that point using the helix parameters.  This is
    // assumed constant for this crude estimate
    double mom = TrkMomCalculator::vecMom(tdef.helix(),bField(),t0flt).mag();
    // compute the particle velocity
    double vflt = tdef.particle().beta(mom)*CLHEP::c_light;
//-----------------------------------------------------------------------------
// Calculate the path length of the particle from the middle of the Tracker to the 
// calorimeter, TPeak->Z() is calculated wrt the tracker center 
//-----------------------------------------------------------------------------
    double path = TPeak->ClusterZ()/tdef.helix().sinDip();

    t0._t0 = TPeak->ClusterT0() + _dtoffset - path/vflt;
    
    //Set dummy error value
    t0._t0err = 1.;
  }


  void
  KalFitHack::initT0(TrkDef const& tdef, TrkT0& t0) {
    using namespace boost::accumulators;
// make an array of all the hit times, correcting for propagation delay
    const Tracker& tracker = getTrackerOrThrow();
    ConditionsHandle<TrackerCalibrations> tcal("ignored");
    unsigned nind = tdef.strawHitIndices().size();
    std::vector<double> times;
    times.reserve(nind);
    // get flight distance of z=0
    double t0flt = tdef.helix().zFlight(0.0);
    // estimate the momentum at that point using the helix parameters.  This is
    // assumed constant for this crude estimate
    double mom = TrkMomCalculator::vecMom(tdef.helix(),bField(),t0flt).mag();
    // compute the particle velocity
    double vflt = tdef.particle().beta(mom)*CLHEP::c_light;
    // for crude estimates, we only need 1 d2t function
    D2T d2t;
    static CLHEP::Hep3Vector zdir(0.0,0.0,1.0);
    // loop over strawhits
    for(unsigned iind=0;iind<nind;iind++){
      size_t istraw = tdef.strawHitIndices()[iind]._index;
      const StrawHit& strawhit(tdef.strawHitCollection()->at(istraw));
      const Straw& straw = tracker.getStraw(strawhit.strawIndex());
      // compute the flightlength to this hit from z=0 (can be negative)
      double hflt = tdef.helix().zFlight(straw.getMidPoint().z()) - t0flt;
      // Use this to estimate the time for the track to reaches this hit from z=0
      double tprop = hflt/vflt;
      // estimate signal propagation time on the wire assuming the middle (average)
      double vwire = tcal->SignalVelocity(straw.index());
      double teprop = straw.getHalfLength()/vwire;
      // correct the measured time for these effects: this gives the aveage time the particle passed this straw, WRT
      // when the track crossed Z=0
    // assume the average drift time is half the maximum drift distance.  This is a poor approximation, but good enough for now
      if(iind==0)tcal->DistanceToTime(straw.index(),0.5*straw.getRadius(),zdir,d2t);
      double htime = strawhit.time() - tprop - teprop - d2t._tdrift;
      times.push_back(htime);
    }
    // find the median time
    accumulator_set<double, stats<tag::median(with_p_square_quantile) > > med;
    med = std::for_each( times.begin(), times.end(), med );
    t0._t0 = extract_result<tag::median>(med);
    accumulator_set<double, stats<tag::min> >  min;
    accumulator_set<double, stats<tag::max> > max;
    min = std::for_each( times.begin(), times.end(), min );
    max = std::for_each( times.begin(), times.end(), max );
    double tmin = extract_result<tag::min>(min);
    double tmax = extract_result<tag::max>(max);
    // estimate the error using the range
    t0._t0err = (tmax-tmin)/sqrt(12*nind);
  }

//-----------------------------------------------------------------------------
// 
//-----------------------------------------------------------------------------
  void KalFitHack::updateCalT0(KalFitResult& kres, CalTimePeak* TPeak) {
//    2014-11-24 gianipez and Pasha removed time offset between caloriemter and tracker

    TrkT0 t0;
    double mom, vflt, path, t0flt, flt0(0.0);
    bool converged = TrkHelixUtils::findZFltlen(kres._krep->traj(),0.0,flt0);
    
    //get helix from kalrep
    HelixTraj trkHel(kres._krep->helix(flt0).params(),kres._krep->helix(flt0).covariance());
    
					// get flight distance of z=0
    t0flt = trkHel.zFlight(0.0);
    
    if (converged) {
//-----------------------------------------------------------------------------
// estimate the momentum at that point using the helix parameters.  
// This is assumed constant for this crude estimate
// compute the particle velocity
//-----------------------------------------------------------------------------
      mom  = TrkMomCalculator::vecMom(trkHel,bField(),t0flt).mag();
      vflt = kres._tdef.particle().beta(mom)*CLHEP::c_light;
//-----------------------------------------------------------------------------
// path length of the particle from the middle of the Tracker to the  calorimeter
// set dummy error value
//-----------------------------------------------------------------------------
      path      = TPeak->ClusterZ()/trkHel.sinDip();
      t0._t0    = TPeak->ClusterT0() + _dtoffset - path/vflt;
      t0._t0err = 1.0;
      
      kres._krep->setT0(t0,flt0);
      updateHitTimes(kres);
    }
  }
  
  bool KalFitHack::updateT0(KalFitResult& kres) {
    using namespace boost::accumulators;
    bool retval(false);
    ConditionsHandle<TrackerCalibrations> tcal("ignored");
    KalRep* krep = kres._krep;
// need to have a valid fit
    if(krep->fitValid()) {
// find the global fltlen associated with z=0. 
      double flt0(0.0);
      bool converged = TrkHelixUtils::findZFltlen(krep->traj(),0.0,flt0);
      if(converged){
	std::vector<double> hitt0; // store t0, to allow outlyer removal
	std::vector<double> hitt0err;
	hitt0.reserve(kres._hits.size());
	hitt0err.reserve(kres._hits.size());
	// loop over the hits
	for(std::vector<TrkStrawHit*>::iterator ihit= kres._hits.begin();ihit != kres._hits.end(); ihit++){
	  TrkStrawHit* hit = *ihit;
	  if(hit->isActive() && hit->poca()!= 0 && hit->poca()->status().success()){
	    // find the residual, exluding this hits measurement
	    double resid,residerr;
	    if(krep->resid(hit,resid,residerr,true)){
	      // convert this to a distance to the wire
	      double doca = (resid + hit->driftRadius()*hit->ambig());
	      if(hit->ambig() == 0)
		doca = fabs(doca);
	      else
		doca *= hit->ambig();
	      // restrict the range, symmetrically to avoid bias
	      double rad = hit->straw().getRadius();
	      if(doca > _mint0doca && doca < rad-_mint0doca){
		// translate the DOCA into a time
		D2T d2t;
		tcal->DistanceToTime(hit->straw().index(),doca,krep->traj().direction(hit->fltLen()),d2t);
		// subtracting hitT0 makes this WRT the previous track t0
		hitt0.push_back(hit->time() - d2t._tdrift - hit->signalTime() - hit->hitT0()._t0);
		// assume residual error dominates
		hitt0err.push_back(residerr/d2t._vdrift);
	      }
	    }
	  }
	}
	if(hitt0.size() >1){
	  TrkT0 t0;
	  // find the median
	  accumulator_set<double, stats<tag::median(with_p_square_quantile) > > med;
	  med = std::for_each( hitt0.begin(), hitt0.end(), med );
	  t0._t0 = extract_result<tag::median>(med);
	  // iterate an outlier search and linear fit until the set of used hits doesn't change
	  bool changed(true);
	  std::vector<bool> used(hitt0.size(),true);
	  unsigned niter(0);
	  while(changed && niter < 10){
	    niter++;
	    changed = false;
	    accumulator_set<double,stats<tag::weighted_variance>,double > wmean;
	    for(unsigned ihit=0;ihit<hitt0.size();ihit++){
	      bool useit = fabs(hitt0[ihit]-t0._t0) < _t0nsig*hitt0err[ihit];
	      changed |= useit != used[ihit];
	      used[ihit] = useit;
	      if(useit){
		wmean(hitt0[ihit], weight=1.0/(hitt0err[ihit]*hitt0err[ihit]));
	      }
	    }
	    unsigned nused = extract_result<tag::count>(wmean);
	    if(nused > 1){
	      t0._t0 = extract_result<tag::weighted_mean>(wmean);
	      t0._t0err = sqrt(extract_result<tag::weighted_variance>(wmean)/nused);
	    } else {
	      break;
	    }
	  }
	  // reset t0
	  if(!changed){
	    // put in t0 from the track.
	    t0._t0 += krep->t0()._t0;
	    krep->setT0(t0,flt0);
	    updateHitTimes(kres);
	    retval = true;
	  }
	}
      }
    }
    return retval;
  }

  void
  KalFitHack::updateHitTimes(KalFitResult& kres) {
  // compute the time the track came closest to the wire for each hit, starting from t0 and working out.
  // this function allows for momentum change along the track.
  // find the bounding hits on either side of this
    std::sort(kres._hits.begin(),kres._hits.end(),fltlencomp(kres._tdef.fitdir().fitDirection()));
    std::vector<TrkStrawHit*>::iterator ihigh;
    std::vector<TrkStrawHit*>::reverse_iterator ilow;
    findBoundingHits(kres._hits,kres._krep->flt0(),ilow,ihigh);
    // reset all the hit times
    double hflt = kres._krep->flt0();
    TrkT0 hitt0 = kres._krep->t0();
    for(std::vector<TrkStrawHit*>::iterator ihit= ihigh;ihit != kres._hits.end(); ++ihit){
      TrkStrawHit* hit = *ihit;
// particle momentum at this point, using the full fit
      double mom = kres._krep->momentum(hit->fltLen()).mag();
// relativistic velocity from that
      double beta = kres._tdef.particle().beta(mom);
// particle transit time to this hit from the reference
      double tflt = (hit->fltLen()-hflt)/(beta*CLHEP::c_light);
// update the time in the TrkT0 object
      hitt0._t0 += tflt;
      (*ihit)->updateHitT0(hitt0);
// update the reference flightlength
      hflt = hit->fltLen();
    }
    if (_debug > 1) {
      printf("[KalFitHack::updateHitTimes] moving forward\n");
      printHits(kres);
    }

// now the same, moving backwards
    hflt = kres._krep->flt0();
    hitt0 = kres._krep->t0();
    for(std::vector<TrkStrawHit*>::reverse_iterator ihit= ilow;ihit != kres._hits.rend(); ++ihit){
      TrkStrawHit* hit = *ihit;
      double mom = kres._krep->momentum(hit->fltLen()).mag();
      double beta = kres._tdef.particle().beta(mom);
      double tflt = (hit->fltLen()-hflt)/(beta*CLHEP::c_light);
      hitt0._t0 += tflt;
      (*ihit)->updateHitT0(hitt0);
      hflt = hit->fltLen();
    }

    if (_debug > 1) {
      printf("[KalFitHack::updateHitTimes] moving backwards\n");
      printHits(kres);
    }

  }

  void
  KalFitHack::findBoundingHits(std::vector<TrkStrawHit*>& hits,double flt0,
    std::vector<TrkStrawHit*>::reverse_iterator& ilow,
    std::vector<TrkStrawHit*>::iterator& ihigh) {
    ilow = hits.rbegin();
    ihigh = hits.begin();
    while(ilow != hits.rend() && (*ilow)->fltLen() > flt0 )++ilow;
    while(ihigh != hits.end() && (*ihigh)->fltLen() < flt0 )++ihigh;
  }

}
