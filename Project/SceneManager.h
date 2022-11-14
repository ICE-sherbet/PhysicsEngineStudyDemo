// @SceneManager.h
// @brief
// @author ICE
// @date 2022/10/11
//
// @details

#pragma once
#include <map>
#include <optional>
#include <ranges>

#include "Scene.h"

namespace base_engine {
namespace scene {
using SceneKey = size_t;

class SceneManager {
  std::map<SceneKey, Scene*> scenes_;
  std::optional<Scene*> current_scene_;

 public:
  bool ChangeScene(SceneKey key);
    //TODO Sceneにアクターを追加する
  void Register()
  {
      
  }
  ~SceneManager();
};
}  // namespace scene
}  // namespace base_engine