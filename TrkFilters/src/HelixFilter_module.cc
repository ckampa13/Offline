//
//  Filter for selecting good helices (pat. rec. output): this is part of the track trigger
//  Original author: Dave Brown (LBNL) 3/1/2017
//
// framework
#include "art/Framework/Core/EDFilter.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "RecoDataProducts/inc/TrkFitFlag.hh"
#include "fhiclcpp/ParameterSet.h"
// mu2e
// data
#include "RecoDataProducts/inc/HelixSeed.hh"
using namespace CLHEP;
// c++
#include <string>
#include <vector>
#include <iostream>

using namespace std;

namespace mu2e 
{
  class HelixFilter : public art::EDFilter 
  {
    public:
      explicit HelixFilter(fhicl::ParameterSet const& pset);
      virtual bool filter(art::Event& event) override;
      virtual void endJob() override;

    private:
      art::InputTag _hsTag;
      bool _hascc; // Calo Cluster
      unsigned _minnhits;
      double _minmom, _maxmom;
      TrkFitFlag _goodh; // helix fit flag
      // counters
      unsigned _nevt, _npass;
  };

  HelixFilter::HelixFilter(fhicl::ParameterSet const& pset) :
    _hsTag(pset.get<art::InputTag>("HelixSeedCollection","PosHelixFinder")),
    _hascc(pset.get<bool>("RequireCaloCluster",false)),
    _minnhits(pset.get<unsigned>("MinNHits",11)),
    _minmom(pset.get<double>("MinMomentum",280.0)),
    _maxmom(pset.get<double>("MaxMomentum",380.0)) ,
    _goodh(pset.get<vector<string> >("HelixFitFlag",vector<string>{"HelixOK"})),
    _nevt(0), _npass(0)
  {
  }

  bool HelixFilter::filter(art::Event& evt){
    ++_nevt;
    bool retval(false); // preset to fail
// find the collection
    auto hsH = evt.getValidHandle<HelixSeedCollection>(_hsTag);
    const HelixSeedCollection* hscol = hsH.product();
// loop over the collection: if any pass the selection, pass this event
    for(auto const& hs : *hscol ) {
    // compute the helix momentum.  Note this is in units of mm!!!
      float hmom = sqrt(hs.helix().radius()*hs.helix().radius() + hs.helix().lambda()*hs.helix().lambda());
 
      if( hs.status().hasAllProperties(_goodh) &&
	  (!_hascc || hs.caloCluster().isNonnull()) &&
	  hs.hits().size() >= _minnhits &&
	  hmom > _minmom && hmom < _maxmom) {
	retval = true;
	++_npass;
// should create a trigger product output FIXME
	break;
      }
    }
    return retval;
  }

  void HelixFilter::endJob() {
    if(_nevt > 0){
      cout << "HelixFilter passed " <<  _npass << " events out of " << _nevt << " for a ratio of " << float(_npass)/float(_nevt) << endl;
    }

  }
}
