// @GameSetup.h
// @brief
// @author ICE
// @date 2022/10/18
// 
// @details

#pragma once

namespace base_engine
{
    class Game;
}

class GameSetup
{
protected:
  base_engine::Game* game_ = nullptr;

public:
  GameSetup(base_engine::Game* game) { game_ = game; }
 void virtual SetUp() = 0;
};
