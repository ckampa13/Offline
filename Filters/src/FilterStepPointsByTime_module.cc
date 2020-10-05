// Copy step points passing time cuts to a new collection.
// Filter module based on similar producer.
//
// Andrei Gaponenko, 2014
// Brian Pollack, 2016

#include <string>
#include <vector>
#include <algorithm>
#include <limits>

// art includes.
#include "fhiclcpp/ParameterSet.h"
#include "art/Framework/Core/EDFilter.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Core/ModuleMacros.h"

#include "MCDataProducts/inc/StepPointMC.hh"
#include "MCDataProducts/inc/StepPointMCCollection.hh"

namespace mu2e {

  //================================================================
  class FilterStepPointsByTime : public art::EDFilter {
    art::InputTag input_;
    std::string outInstanceName_;
    double cutTimeMin_;
    double cutTimeMax_;
    double cutTimeMinUnc_; //to allow for a window around tmin

    // statistics
    unsigned numInputHits_;
    unsigned numOutputHits_;

    // for debugging
    int verbosityLevel_;

  public:
    explicit FilterStepPointsByTime(const fhicl::ParameterSet& pset);
    virtual bool filter(art::Event& event) override;
    virtual void endJob() override;
  };

  //================================================================
  FilterStepPointsByTime::FilterStepPointsByTime(const fhicl::ParameterSet& pset)
    : art::EDFilter(pset),
      input_(pset.get<std::string>("input"))
    , outInstanceName_(pset.get<std::string>("outInstanceName"))
    , cutTimeMin_(pset.get<double>("cutTimeMin",std::numeric_limits<double>::min()))
    , cutTimeMax_(pset.get<double>("cutTimeMax",std::numeric_limits<double>::max()))
    , cutTimeMinUnc_(pset.get<double>("cutTimeMinUnc",0.))
    , numInputHits_(0)
    , numOutputHits_(0)
    , verbosityLevel_(pset.get<int>("verbosityLevel", 1))
  {
    produces<StepPointMCCollection>(outInstanceName_);
    if(verbosityLevel_ > 0) mf::LogInfo("Info") <<"FilterStepPointsByTime: cuts for "
						<<input_
						<<" are: min = "<<cutTimeMin_ 
						<<"(within " << cutTimeMinUnc_ <<")"
						<<", max = "<<cutTimeMax_
						<<"\n";
  }

  //================================================================
  bool FilterStepPointsByTime::filter(art::Event& event) {
    bool passed = false;
    std::unique_ptr<StepPointMCCollection> out(new StepPointMCCollection());

    auto ih = event.getValidHandle<StepPointMCCollection>(input_);
    for(const auto& hit : *ih) {
      double time = hit.time();
      if( (cutTimeMin_-abs(cutTimeMinUnc_) < time) && (time < cutTimeMax_)) {
	out->emplace_back(hit);
	passed = true;
      }
      if(verbosityLevel_ > 9) std::cout << "Found hit with time " << time << "\n";
    }

    numInputHits_ += ih->size();
    numOutputHits_ += out->size();

    event.put(std::move(out), outInstanceName_);
    if(verbosityLevel_ > 1) std::cout << "Filtered event has status " 
					       << passed << "\n";
    return passed;

  }

  //================================================================
  void FilterStepPointsByTime::endJob() {
    if(verbosityLevel_ > 0) mf::LogInfo("Summary")<<"FilterStepPointsByTime: passed "
						  <<numOutputHits_ <<" / "<<numInputHits_
						  <<" StepPointMCs\n";
  }

  //================================================================
} // namespace mu2e

DEFINE_ART_MODULE(mu2e::FilterStepPointsByTime);
