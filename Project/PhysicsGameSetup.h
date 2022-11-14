// @PhysicsGameSetup.h
// @brief
// @author ICE
// @date 2022/10/18
//
// @details

#pragma once
#include "GameSetup.h"

class PhysicsGameSetup : public GameSetup {
 public:
    explicit PhysicsGameSetup(base_engine::Game* game)
        : GameSetup(game)
    {
    }

    void SetUp() override;
};
