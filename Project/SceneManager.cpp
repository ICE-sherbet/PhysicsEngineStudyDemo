#include "SceneManager.h"

bool base_engine::scene::SceneManager::ChangeScene(SceneKey key) {
  const auto itr = scenes_.find(key);
  if (itr == scenes_.end()) {
    return false;
  }
  if (current_scene_) current_scene_.value()->Clear();
  current_scene_ = itr->second;
  return true;
}

base_engine::scene::SceneManager::~SceneManager() {
  for (const auto val : scenes_ | std::views::values) {
    delete val;
  }
}
