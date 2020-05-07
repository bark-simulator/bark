// #ifndef MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_
// #define MODULES_WORLD_EVALUATION_EVALUATOR_RSS_HPP_

#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include <ad/map/landmark/LandmarkIdSet.hpp>
#include <ad/map/match/Object.hpp>
#include <ad/map/route/FullRoute.hpp>
#include <ad/rss/core/RssCheck.hpp>
#include <ad/rss/map/RssSceneCreation.hpp>
#include <ad/rss/situation/SituationSnapshot.hpp>
#include <ad/rss/state/ProperResponse.hpp>
#include <ad/rss/state/RssStateSnapshot.hpp>