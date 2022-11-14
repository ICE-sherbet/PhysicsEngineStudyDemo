// @Scene.h
// @brief
// @author ICE
// @date 2022/10/11
//
// @details

#pragma once
#include <memory>
#include <vector>

namespace base_engine {
using ActorPtr = std::shared_ptr<class Actor>;
using ActorWeakPtr = std::weak_ptr<class Actor>;
constexpr size_t capacity = 255;
namespace scene {
class Scene {
  std::vector<ActorWeakPtr> actors_;

 public:
  Scene() { actors_.reserve(capacity); }
  void Clear() const;
};

}  // namespace scene
}  // namespace base_engine