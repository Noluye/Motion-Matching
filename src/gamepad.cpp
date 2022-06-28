#include "mmpch.h"
#include "gamepad.h"


vec3 gamepad_get_stick(int stick, const float deadzone) {
	float gamepadx = GetGamepadAxisMovement(GAMEPAD_PLAYER, stick == GAMEPAD_STICK_LEFT ? GAMEPAD_AXIS_LEFT_X : GAMEPAD_AXIS_RIGHT_X);
	float gamepady = GetGamepadAxisMovement(GAMEPAD_PLAYER, stick == GAMEPAD_STICK_LEFT ? GAMEPAD_AXIS_LEFT_Y : GAMEPAD_AXIS_RIGHT_Y);
	float gamepadmag = sqrtf(gamepadx * gamepadx + gamepady * gamepady);

	if (gamepadmag > deadzone)
	{
		float gamepaddirx = gamepadx / gamepadmag;
		float gamepaddiry = gamepady / gamepadmag;
		float gamepadclippedmag = gamepadmag > 1.0f ? 1.0f : gamepadmag * gamepadmag;
		gamepadx = gamepaddirx * gamepadclippedmag;
		gamepady = gamepaddiry * gamepadclippedmag;
	}
	else
	{
		gamepadx = 0.0f;
		gamepady = 0.0f;
	}

	return vec3(gamepadx, 0.0f, gamepady);
}
