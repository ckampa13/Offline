//
// Prototype analysis module using tracks.  This module associates information from the
// Mu2e detector systems into a single coherent analysis TTree (trkana).  This module
// depends on the data products produced by reconstruction and (if requested) their MC
// counterparts.  The primary analysis object is the set of Downstream electron track fits.
// Upstream electron fit and downstream muon are also required for PID and quality selection.
// Calorimeter clusters and Track-cluster matching are used for PID. CRV coincidences are also
// included for rejecting cosmic backgrounds.
// Most of the calcluations are done by upstream modules and helper classes.
// Original author: Dave Brown (LBNL) 7/7/2016
// Updated November 2018 to run on KalSeeds only (A. Edmonds)
//

// Mu2e includes
#include "GeneralUtilities/inc/ParameterSetHelpers.hh"
#include "MCDataProducts/inc/ProtonBunchIntensity.hh"
#include "MCDataProducts/inc/EventWeight.hh"
#include "MCDataProducts/inc/KalSeedMC.hh"
#include "MCDataProducts/inc/CaloClusterMC.hh"
#include "RecoDataProducts/inc/CaloCrystalHit.hh"
#include "TrkReco/inc/TrkUtilities.hh"
#include "CalorimeterGeom/inc/DiskCalorimeter.hh"
#include "GeometryService/inc/VirtualDetector.hh"
// Framework includes.
#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Services/Optional/TFileService.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "canvas/Persistency/Common/TriggerResults.h"

// ROOT incldues
#include "Rtypes.h"
#include "TBits.h"
#include "TTree.h"
#include "TProfile.h"

// BaBar includes
#include "BTrk/BaBar/BaBar.hh"
#include "BTrk/KalmanTrack/KalRep.hh"
#include "BTrk/ProbTools/ChisqConsistency.hh"
#include "BTrk/BbrGeom/BbrVectorErr.hh"
#include "BTrk/TrkBase/TrkHelixUtils.hh"
#include "Mu2eUtilities/inc/TriggerResultsNavigator.hh"
#include "Mu2eUtilities/inc/SimParticleTimeOffset.hh"
// mu2e tracking
#include "RecoDataProducts/inc/TrkFitDirection.hh"
#include "BTrkData/inc/TrkStrawHit.hh"
// diagnostics
#include "TrkDiag/inc/TrkComp.hh"
#include "TrkDiag/inc/HitCount.hh"
#include "TrkDiag/inc/TrkCount.hh"
#include "TrkDiag/inc/EventInfo.hh"
#include "TrkDiag/inc/TrkInfo.hh"
#include "TrkDiag/inc/GenInfo.hh"
#include "TrkDiag/inc/EventWeightInfo.hh"
#include "TrkDiag/inc/TrkStrawHitInfo.hh"
#include "TrkDiag/inc/TrkStrawHitInfoMC.hh"
#include "TrkDiag/inc/TrkCaloHitInfo.hh"
#include "TrkDiag/inc/CaloClusterInfoMC.hh"
#include "TrkDiag/inc/TrkQualInfo.hh"
#include "TrkDiag/inc/TrkQualTestInfo.hh"
#include "TrkDiag/inc/HelixInfo.hh"
#include "TrkDiag/inc/InfoStructHelper.hh"
#include "TrkDiag/inc/InfoMCStructHelper.hh"
// CRV info
#include "CRVAnalysis/inc/CRVAnalysis.hh"

// C++ includes.
#include <iostream>
#include <string>
#include <cmath>

using namespace std;

namespace mu2e {
// Need this for the BaBar headers.
  using CLHEP::Hep3Vector;
  typedef KalSeedCollection::const_iterator KSCIter;

  class TrackAnalysisReco : public art::EDAnalyzer {

  public:

    struct Config {
      using Name=fhicl::Name;
      using Comment=fhicl::Comment;

      fhicl::Atom<art::InputTag> detag{Name("DeTag"), Comment("KalSeedCollection for De")};
      fhicl::Atom<art::InputTag> uetag{Name("UeTag"), Comment("KalSeedCollection for Ue")};
      fhicl::Atom<art::InputTag> dmtag{Name("DmuTag"), Comment("KalSeedCollection for Dmu")};
      fhicl::Atom<art::InputTag> detqtag{Name("DeTrkQualTag"), Comment("TrkQualCollection for De")};
      fhicl::Atom<art::InputTag> rctag{Name("RecoCountTag"), Comment("RecoCount"), art::InputTag()};
      fhicl::Atom<art::InputTag> cchmtag{Name("CaloCrystalHitMapTag"), Comment("CaloCrystalHitMapTag"), art::InputTag()};
      fhicl::Atom<art::InputTag> meanPBItag{Name("MeanBeamIntensity"), Comment("Tag for MeanBeamIntensity"), art::InputTag()};
      fhicl::Atom<art::InputTag> PBIwtTag{Name("PBIWeightTag"), Comment("Tag for PBIWeight") ,art::InputTag()};
      fhicl::Atom<std::string> crvCoincidenceModuleLabel{Name("CrvCoincidenceModuleLabel"), Comment("CrvCoincidenceModuleLabel")};
      fhicl::Atom<std::string> crvCoincidenceMCModuleLabel{Name("CrvCoincidenceMCModuleLabel"), Comment("CrvCoincidenceMCModuleLabel")};
      fhicl::Atom<bool> fillmc{Name("FillMCInfo"),true};
      fhicl::Atom<bool> pempty{Name("ProcessEmptyEvents"),false};
      fhicl::Atom<bool> crv{Name("AnalyzeCRV"),false};
      fhicl::Atom<bool> helices{Name("FillHelixInfo"),false};
      fhicl::Atom<bool> filltrkqual{Name("FillTrkQualInfo"),true};
      fhicl::Atom<bool> filltrig{Name("FillTriggerInfo"),false};
      fhicl::Atom<int> diag{Name("diagLevel"),1};
      fhicl::Atom<int> debug{Name("debugLevel"),0};
      fhicl::Atom<double> minReflectTime{Name("MinimumReflectionTime"), Comment("ns"),20}; // nsec
      fhicl::Atom<double> maxReflectTime{Name("MaximumReflectionTime"), Comment("ns"),200}; // nsec
      fhicl::Atom<art::InputTag> primaryParticleTag{Name("PrimaryParticleTag"), Comment("Tag for PrimaryParticle"), art::InputTag()};
      fhicl::Atom<art::InputTag> kalSeedMCTag{Name("KalSeedMCAssns"), Comment("Tag for KalSeedMCAssn"), art::InputTag()};
      fhicl::Atom<art::InputTag> caloClusterMCTag{Name("CaloClusterMCAssns"), Comment("Tag for CaloClusterMCAssns"), art::InputTag()};
      fhicl::Table<InfoMCStructHelper::Config> infoMCStructHelper{Name("InfoMCStructHelper"), Comment("Configuration for the InfoMCStructHelper")};
    };
    typedef art::EDAnalyzer::Table<Config> Parameters;

    explicit TrackAnalysisReco(const Parameters& conf);
    virtual ~TrackAnalysisReco() { }

    void beginJob();
    void beginSubRun(const art::SubRun & subrun ) override;
    void analyze(const art::Event& e);

  private:

    Config _conf;

    // track comparator
    TrkComp _tcomp;
    // main TTree
    TTree* _trkana;
    TProfile* _tht; // profile plot of track hit times: just an example
    // general event info branch
    double _meanPBI;
    EventInfo _einfo;
    EventWeightInfo _wtinfo;
    // hit counting
    HitCount _hcnt;
    // track counting
    TrkCount _tcnt;
    // track branches
    TrkInfo _deti, _ueti, _dmti;
    TrkFitInfo _deentti, _demidti, _dexitti;
    // detailed info branches for the signal candidate
    std::vector<TrkStrawHitInfo> _detsh;
    art::InputTag _strawHitFlagTag;
    TrkCaloHitInfo _detch;
    CaloClusterInfoMC _detchmc;
    std::vector<TrkStrawMatInfo> _detsm;
    // trigger information
    unsigned _trigbits;
    // MC truth branches
    TrkInfoMC _demc, _uemc, _dmmc;
    art::InputTag _primaryParticleTag;
    art::InputTag _kalSeedMCTag, _caloClusterMCTag;
    std::vector<int> _entvids, _midvids, _xitvids;

    // detailed MC truth for the signal candidate
    GenInfo _demcgen, _demcpri; // generator and 'primary' information
    TrkInfoMCStep _demcent, _demcmid, _demcxit;
    std::vector<TrkStrawHitInfoMC> _detshmc;
    // test trkqual variable branches
    TrkQualInfo _trkQualInfo;
    TrkQualTestInfo _trkqualTest;
    // helper functions
    void fillEventInfo(const art::Event& event);
    void fillTriggerBits(const art::Event& event,std::string const& process);
//    TrkQualCollection const& tqcol, TrkQual& tqual);
    void resetBranches();
    KSCIter findBestRecoTrack(KalSeedCollection const& kcol);
    KSCIter findUpstreamTrack(KalSeedCollection const& kcol,KalSeed const& dekseed);
    KSCIter findMuonTrack(KalSeedCollection const& kcol,KalSeed const& dekseed);
    // CRV info
    std::vector<CrvHitInfoReco> _crvinfo;
    HelixInfo _hinfo;
    std::vector<CrvHitInfoMC> _crvinfomc;
    // SimParticle timing offset
    InfoStructHelper _infoStructHelper;
    InfoMCStructHelper _infoMCStructHelper;
};

  TrackAnalysisReco::TrackAnalysisReco(const Parameters& conf):
    art::EDAnalyzer(conf),
    _conf(conf()),
    _infoMCStructHelper(conf().infoMCStructHelper())
  {
    _midvids.push_back(VirtualDetectorId::TT_Mid);
    _midvids.push_back(VirtualDetectorId::TT_MidInner);
    _entvids.push_back(VirtualDetectorId::TT_FrontHollow);
    _entvids.push_back(VirtualDetectorId::TT_FrontPA);
    _xitvids.push_back(VirtualDetectorId::TT_Back);
  }

  void TrackAnalysisReco::beginJob( ){
    art::ServiceHandle<art::TFileService> tfs;
// create TTree
    _trkana=tfs->make<TTree>("trkana","track analysis");
    _tht=tfs->make<TProfile>("tht","Track Hit Time Profile",RecoCount::_nshtbins,-25.0,1725.0);
// add event info branch
    _trkana->Branch("evtinfo.",&_einfo,EventInfo::leafnames().c_str());
// hit counting branch
    _trkana->Branch("hcnt.",&_hcnt,HitCount::leafnames().c_str());
// track counting branch
    _trkana->Branch("tcnt.",&_tcnt,TrkCount::leafnames().c_str());
// add primary track (downstream electron) branch
    _trkana->Branch("de.",&_deti,TrkInfo::leafnames().c_str());
    _trkana->Branch("deent",&_deentti,TrkFitInfo::leafnames().c_str());
    _trkana->Branch("demid",&_demidti,TrkFitInfo::leafnames().c_str());
    _trkana->Branch("dexit",&_dexitti,TrkFitInfo::leafnames().c_str());
    //
    _trkana->Branch("detch",&_detch,TrkCaloHitInfo::leafnames().c_str());
// optionally add detailed branches
    if(_conf.diag() > 1){
      _trkana->Branch("detsh",&_detsh);
      _trkana->Branch("detsm",&_detsm);
    }
// add branches for other tracks
    _trkana->Branch("ue.",&_ueti,TrkInfo::leafnames().c_str());
    _trkana->Branch("dm.",&_dmti,TrkInfo::leafnames().c_str());
// trigger info.  Actual names should come from the BeginRun object FIXME
    if(_conf.filltrig())_trkana->Branch("trigbits",&_trigbits,"trigbits/i");
// calorimeter information for the downstream electron track
// CRV info
    if(_conf.crv()) _trkana->Branch("crvinfo",&_crvinfo);
   // helix info
   if(_conf.helices()) _trkana->Branch("helixinfo",&_hinfo,HelixInfo::leafnames().c_str());
// optionally add MC truth branches
    if(_conf.fillmc()){
      _trkana->Branch("demc",&_demc,TrkInfoMC::leafnames().c_str());
      _trkana->Branch("demcgen",&_demcgen,GenInfo::leafnames().c_str());
      _trkana->Branch("demcpri",&_demcpri,GenInfo::leafnames().c_str());
      _trkana->Branch("demcent",&_demcent,TrkInfoMCStep::leafnames().c_str());
      _trkana->Branch("demcmid",&_demcmid,TrkInfoMCStep::leafnames().c_str());
      _trkana->Branch("demcxit",&_demcxit,TrkInfoMCStep::leafnames().c_str());
      if(_conf.crv())_trkana->Branch("crvinfomc",&_crvinfomc);
      _trkana->Branch("detchmc",&_detchmc,CaloClusterInfoMC::leafnames().c_str());
      if(_conf.diag() > 1)_trkana->Branch("detshmc",&_detshmc);
    }
    if (_conf.filltrkqual()) {
      _trkana->Branch("detrkqual", &_trkQualInfo, TrkQualInfo::leafnames().c_str());
    }
  }

  void TrackAnalysisReco::beginSubRun(const art::SubRun & subrun ) {
    // mean number of protons on target
    art::Handle<ProtonBunchIntensity> PBIHandle;
    subrun.getByLabel(_conf.meanPBItag(), PBIHandle);
    if(PBIHandle.isValid())
      _meanPBI = PBIHandle->intensity();
    // get bfield
    _infoStructHelper.updateSubRun();
  }

  void TrackAnalysisReco::analyze(const art::Event& event) {
    // update timing maps
    if(_conf.fillmc()){
      _infoMCStructHelper.updateEvent(event);
    }

    // need to create and define the event weight branch here because we only now know the EventWeight creating modules that have been run through the Event
    if (!_trkana->GetBranch("evtwt")) { 
      std::vector<art::Handle<EventWeight> > eventWeightHandles;
      event.getManyByType(eventWeightHandles);
      if (eventWeightHandles.size()>0) {
	std::vector<std::string> labels;
	for (const auto& i_weightHandle : eventWeightHandles) {
	  std::string moduleLabel = i_weightHandle.provenance()->moduleLabel();
	  std::string instanceName = i_weightHandle.provenance()->productInstanceName();

	  std::string branchname = moduleLabel;
	  if (instanceName != "") {
	    branchname += "_" + instanceName;
	  }
	  labels.push_back(branchname);
	}
	_trkana->Branch("evtwt",&_wtinfo,_wtinfo.leafnames(labels).c_str());
      }
    }

    // Get handle to downstream electron track collection.  This also creates the final set of hit flags
    art::Handle<KalSeedCollection> deH;
    event.getByLabel(_conf.detag(),deH);
    // get the provenance from this for trigger processing
    std::string const& process = deH.provenance()->processName();
    // std::cout << _conf.detag() << std::endl; //teste
    auto const& deC = *deH;
    // find downstream muons and upstream electrons
    art::Handle<KalSeedCollection> ueH;
    event.getByLabel(_conf.uetag(),ueH);
    auto const& ueC = *ueH;
    art::Handle<KalSeedCollection> dmH;
    event.getByLabel(_conf.dmtag(),dmH);
    auto const& dmC = *dmH;
    art::Handle<CaloCrystalHitRemapping> cchmH;
    event.getByLabel(_conf.cchmtag(),cchmH);
    auto const& cchmap = *cchmH;
    // general reco counts
    auto rch = event.getValidHandle<RecoCount>(_conf.rctag());
    auto const& rc = *rch;
    for(size_t ibin=0;ibin < rc._nshtbins; ++ibin){
      float time = rc._shthist.binMid(ibin);
      float count  = rc._shthist.binContents(ibin);
      _tht->Fill(time,count);
    }
    // TrkQualCollection
    art::Handle<TrkQualCollection> trkQualHandle;
    event.getByLabel(_conf.detqtag(), trkQualHandle);
    TrkQualCollection const& tqcol = *trkQualHandle;
    // trigger information
    if(_conf.filltrig()){
      fillTriggerBits(event,process);
    }
    // MC data
    art::Handle<PrimaryParticle> pph;
    art::Handle<KalSeedMCAssns> ksmcah;
    art::Handle<CaloClusterMCAssns> ccmcah;
    if(_conf.fillmc()) { // get MC product collections
      event.getByLabel(_conf.primaryParticleTag(),pph);
      event.getByLabel(_conf.kalSeedMCTag(),ksmcah);
      event.getByLabel(_conf.caloClusterMCTag(),ccmcah);
    }
    // reset
    resetBranches();
    // loop through all tracks
    auto idekseed = findBestRecoTrack(deC);
    // process the best track
    if (idekseed != deC.end()) {
      auto const&  dekseed = *idekseed;
      _infoStructHelper.fillTrkInfo(dekseed,_deti);
      // TODO: get entpos from virtualdetector
      mu2e::GeomHandle<VirtualDetector> vdHandle;
      mu2e::GeomHandle<DetectorSystem> det;
      const XYZVec& entpos = XYZVec(det->toDetector(vdHandle->getGlobal(*_entvids.begin())));
      const XYZVec& midpos = XYZVec(det->toDetector(vdHandle->getGlobal(*_midvids.begin())));
      const XYZVec& xitpos = XYZVec(det->toDetector(vdHandle->getGlobal(*_xitvids.begin())));
      _infoStructHelper.fillTrkFitInfo(dekseed,_deentti,entpos);
      _infoStructHelper.fillTrkFitInfo(dekseed,_demidti,midpos);
      _infoStructHelper.fillTrkFitInfo(dekseed,_dexitti,xitpos);
      if(_conf.diag() > 1){
	_infoStructHelper.fillHitInfo(dekseed, _detsh);
	_infoStructHelper.fillMatInfo(dekseed, _detsm);
      }
      if(_conf.helices())_infoStructHelper.fillHelixInfo(dekseed, _hinfo);
      // upstream and muon tracks
      auto iuekseed = findUpstreamTrack(ueC,dekseed);
      if(iuekseed != ueC.end()) _infoStructHelper.fillTrkInfo(*iuekseed,_ueti);
      auto idmukseed = findMuonTrack(dmC,dekseed);
      if(idmukseed != dmC.end()) _infoStructHelper.fillTrkInfo(*idmukseed,_dmti);
      // calorimeter info
      if (dekseed.hasCaloCluster()) {
	_infoStructHelper.fillCaloHitInfo(dekseed,  _detch);
	_tcnt._ndec = 1; // only 1 possible calo hit at the moment
	// test
	if(_conf.debug()>0){
	  auto const& tch = dekseed.caloHit();
	  auto const& cc = tch.caloCluster();
	  std::cout << "CaloCluster has energy " << cc->energyDep()
	  << " +- " << cc->energyDepErr() << std::endl;
	  for( auto const& cchptr: cc->caloCrystalHitsPtrVector() ) { 
	    // map the crystal ptr to the reduced collection
	    auto ifnd = cchmap.find(cchptr);
	    if(ifnd != cchmap.end()){
	      auto const& scchptr = ifnd->second;
	      if(scchptr.isNonnull())
		std::cout << "CaloCrystalHit has " << scchptr->energyDep() << " energy Dep" << std::endl;
	      else
		std::cout <<"CalCrystalHitPtr is invalid! "<< std::endl;
	    } else {
	      std::cout << "CaloCrystaLhitPtr not in map!" << std::endl;
	    }
	  }
	}
      }
      if (_conf.filltrkqual()) {
	auto const& tqual = tqcol.at(std::distance(deC.begin(),idekseed));
	_deti._trkqual = tqual.MVAOutput();
	_infoStructHelper.fillTrkQualInfo(tqual, _trkQualInfo);
      }
      // fill mC info associated with this track
      if(_conf.fillmc() ) { 
	const PrimaryParticle& primary = *pph;
	// use Assns interface to find the associated KalSeedMC; this uses ptrs
	auto dekptr = art::Ptr<KalSeed>(deH,std::distance(deC.begin(),idekseed));
	//	std::cout << "KalSeedMCMatch has " << ksmcah->size() << " entries" << std::endl;
	for(auto iksmca = ksmcah->begin(); iksmca!= ksmcah->end(); iksmca++){
	//	  std::cout << "KalSeed Ptr " << dekptr << " match Ptr " << iksmca->first << std::endl;
	  if(iksmca->first == dekptr) {
	    auto const& dekseedmc = *(iksmca->second);
	    _infoMCStructHelper.fillTrkInfoMC(dekseedmc, _demc);
	    _infoMCStructHelper.fillTrkInfoMCStep(dekseedmc, _demcent, _entvids);
	    _infoMCStructHelper.fillTrkInfoMCStep(dekseedmc, _demcmid, _midvids);
	    _infoMCStructHelper.fillTrkInfoMCStep(dekseedmc, _demcxit, _xitvids);
	    _infoMCStructHelper.fillGenAndPriInfo(dekseedmc, primary, _demcpri, _demcgen);

	    if (_conf.diag()>1) {
	      _infoMCStructHelper.fillHitInfoMCs(dekseedmc, _detshmc);
	    }
	    break;
	  }
	}
	if (dekseed.hasCaloCluster()) {
	  // fill MC truth of the associated CaloCluster 
	  for(auto iccmca= ccmcah->begin(); iccmca != ccmcah->end(); iccmca++){
	    if(iccmca->first == dekseed.caloCluster()){
	      auto const& ccmc = *(iccmca->second);
	      _infoMCStructHelper.fillCaloClusterInfoMC(ccmc,_detchmc);

	      break;
	    }
	  }
	}
      }
    }
    if(idekseed != deC.end() || _conf.pempty()) {
      // fill general event information
      fillEventInfo(event);
      _infoStructHelper.fillHitCount(rc, _hcnt);
      // TODO we want MC information when we don't have a track
      // fill CRV info
      if(_conf.crv()) CRVAnalysis::FillCrvHitInfoCollections(_conf.crvCoincidenceModuleLabel(), _conf.crvCoincidenceMCModuleLabel(), event, _crvinfo, _crvinfomc);
      // fill this row in the TTree
      _trkana->Fill();
    }
  }

  KSCIter TrackAnalysisReco::findBestRecoTrack(KalSeedCollection const& kcol) {
    KSCIter retval = kcol.end();
    _tcnt._nde = kcol.size();
    // find the higest momentum track; should be making some quality cuts too FIXME!
    double max_momentum = -9999;
    for(auto i_kseed=kcol.begin(); i_kseed != kcol.end(); ++i_kseed) {
      auto const& kseed = *i_kseed; 
      double this_momentum = kseed.segments().begin()->mom();
      if (this_momentum > max_momentum) {
	retval = i_kseed;
	max_momentum = this_momentum;
      }
    }
    return retval;
  }

  KSCIter TrackAnalysisReco::findUpstreamTrack(KalSeedCollection const& kcol,const KalSeed& dekseed) {
    KSCIter retval = kcol.end();
    _tcnt._nue = kcol.size();
    // loop over upstream tracks and pick the best one (closest to momentum) that's earlier than the downstream track
    double demom = dekseed.segments().begin()->mom();
    double closest_momentum = 0;
    for(auto i_kseed=kcol.begin(); i_kseed != kcol.end(); i_kseed++) {
      if(i_kseed->t0().t0() < dekseed.t0().t0() - _conf.minReflectTime() &&
	 i_kseed->t0().t0() > dekseed.t0().t0() - _conf.maxReflectTime()) {
	double this_ue_momentum = i_kseed->segments().begin()->mom();
// choose the upstream track whose parameters best match the downstream track.
// Currently compare momentum at the tracker center, this should be done at the tracker entrance
// and should compare more parameters FIXME!
	if( fabs(this_ue_momentum-demom) < fabs(closest_momentum-demom)) {
	  retval = i_kseed;
	}
      }
    }
    return retval;
  }

  KSCIter TrackAnalysisReco::findMuonTrack(KalSeedCollection const& kcol,const KalSeed& dekseed) {
    KSCIter retval = kcol.end();
    _tcnt._ndm = kcol.size();
// loop over muon tracks and pick the one with the largest hit overlap
    unsigned maxnover(0);
    for(auto i_kseed = kcol.begin(); i_kseed != kcol.end(); i_kseed++) {
      unsigned nover = _tcomp.nOverlap(*i_kseed,dekseed);
      if(nover > maxnover){
	maxnover = nover;
	retval = i_kseed;
      }
    }
    _tcnt._ndmo = maxnover;
    return retval;
  }

  void TrackAnalysisReco::fillEventInfo( const art::Event& event) {
    // fill basic event information
    _einfo._eventid = event.event();
    _einfo._runid = event.run();
    _einfo._subrunid = event.subRun();

    // get event weight products
    std::vector<art::Handle<EventWeight> > eventWeightHandles;
    event.getManyByType(eventWeightHandles);
    std::vector<Float_t> weights;
    for (const auto& i_weightHandle : eventWeightHandles) {
      double weight = i_weightHandle->weight();
      if (i_weightHandle.provenance()->moduleLabel() == _conf.PBIwtTag().label()) {
	if (_meanPBI > 0.0){
	  _einfo._nprotons = _meanPBI*weight;
	}
	else {
	  _einfo._nprotons = 1; // for non-background mixed jobs
	}
      }
      weights.push_back(weight);
    }
    _wtinfo.setWeights(weights);
  }

  void TrackAnalysisReco::fillTriggerBits(const art::Event& event,std::string const& process) {
    //get the TriggerResult from the process that created the KalFinalFit downstream collection
    art::InputTag const tag{Form("TriggerResults::%s", process.c_str())};
    auto trigResultsH = event.getValidHandle<art::TriggerResults>(tag);
    const art::TriggerResults* trigResults = trigResultsH.product();
    _trigbits = 0;
    for(size_t id=0;id < trigResults->size(); ++id){
      if (trigResults->accept(id)) {
	_trigbits |= 1 << id;
      }
    }
    if(_conf.debug() > 0){
      cout << "Found TriggerResults for process " << process << " with " << trigResults->size() << " Lines"
      << " trigger bits word " << _trigbits << endl;
      TriggerResultsNavigator tnav(trigResults);
      tnav.print();
    }
    
  }

  void TrackAnalysisReco::resetBranches() {
    // reset structs
    _einfo.reset();
    _hcnt.reset();
    _tcnt.reset();
    _deti.reset();
    _deentti.reset();
    _demidti.reset();
    _dexitti.reset();
    _ueti.reset();
    _dmti.reset();
    _hinfo.reset();
    _demc.reset();
    _uemc.reset();
    _dmmc.reset();
    _demcgen.reset();
    _demcpri.reset();
    _demcent.reset();
    _demcmid.reset();
    _demcxit.reset();
    _wtinfo.reset();
    _trkqualTest.reset();
    _trkQualInfo.reset();
    _detch.reset();
    _detchmc.reset();
// clear vectors
    _detsh.clear();
    _detsm.clear();
    _detshmc.clear();
    _crvinfo.clear();
    _crvinfomc.clear();
  }

}  // end namespace mu2e

// Part of the magic that makes this class a module.
// create an instance of the module.  It also registers
using mu2e::TrackAnalysisReco;
DEFINE_ART_MODULE(TrackAnalysisReco);
