#include "stdafx.h"
#include "cellGem.h"

#include "cellCamera.h"
#include "Emu/IdManager.h"
#include "Emu/System.h"
#include "Emu/Cell/PPUModule.h"
#include "pad_thread.h"

#include "psmove.h"
#include "psmove_tracker.h"

#include <climits>

logs::channel cellGem("cellGem");

// ************************
// * HLE helper functions *
// ************************

void gem_t::reset_controller(gsl::not_null<gem_t*> gem, u32 gem_num)
{
	switch (g_cfg.io.move)
	{
	default:
	case move_handler::null:
	{
		connected_controllers = 0;
		break;
	}
	case move_handler::fake:
	{
		connected_controllers = 1;
		break;
	}
	case move_handler::move:
	{
		connected_controllers = gem->connected_controllers;
		break;
	}
	}

	// Assign status and port number
	if (gem_num < connected_controllers)
	{
		controllers[gem_num].status = CELL_GEM_STATUS_READY;
		controllers[gem_num].port = 7u - gem_num;
	}
	else
	{
		controllers[gem_num].status = CELL_GEM_STATUS_DISCONNECTED;
		controllers[gem_num].port = 0;
	}
}

template <>
void fmt_class_string<move_handler>::format(std::string& out, u64 arg)
{
	format_enum(out, arg, [](auto value)
	{
		switch (value)
		{
		case move_handler::null: return "Null";
		case move_handler::fake: return "Fake";
		case move_handler::move: return "PSMove";
		}

		return unknown;
	});
}

/**
 * \brief Verifies that a Move controller id is valid
 * \param gem_num Move controler ID to verify
 * \return True if the ID is valid, false otherwise
 */
static bool check_gem_num(const u32 gem_num)
{
	return gem_num >= 0 && gem_num < CELL_GEM_MAX_NUM;
}

namespace move
{

namespace psmoveapi
{

static void init(gsl::not_null<gem_t*> gem)
{
	if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
		fmt::throw_exception("Couldn't initialize PSMoveAPI");
		// TODO: Don't die
	}

	// const auto g_psmove = fxm::make<psmoveapi_thread>();

	gem->connected_controllers = psmove_count_connected();

	for (auto id = 0; id < gem->connected_controllers; ++id)
	{
		PSMove* connected_controller = psmove_connect_by_id(id);

		if (connected_controller)
		{
			const std::string serial = psmove_get_serial(connected_controller);

			gem->controllers[id].psmove_serial = serial;
			gem->controllers[id].psmove_handle.reset(connected_controller);

			// g_psmove->register_controller(id, connected_controller);
		}
	}
}

enum class poll_result
{
	fail,
	success,
	too_soon,
};

poll_result poll(PSMove* controller_handle)
{

	/*
	auto seq = psmove_poll(controller_handle);

	if (!seq)
	{
		cellGem.fatal("psmoveapi: %02d %02d polling failed", seq_old, seq);
		poll_success = false;
	}
	else
	{
		if ((seq_old > 0) && ((seq_old % 16) != (seq - 1))) {
			cellGem.fatal("psmoveapi: %02d %02d dropped frames", seq_old, seq);
			poll_success = false;
		}
		else
		{
			cellGem.error("psmoveapi: %02d %02d success", seq_old, seq);
		}
	}
	seq_old = seq;
	*/

	thread_local std::uint8_t seq_old = 0;
	thread_local auto last_poll = std::chrono::high_resolution_clock::now();

	const auto now = std::chrono::high_resolution_clock::now();
	if (now - last_poll < 10ms) {
		// This is needed because games call functions like cellGemGetState and cellGemGetInertialState
		// in quick succession, resulting successive polls, resulting in polls after the first one failing
		// ...resulting in our implementation supplying the game with the previous state (state preservation)
		// instead of translating current button state.
		// We can't remove state buffering/preservation either, because if polling fails
		return poll_result::too_soon;
	}

	auto result = poll_result::fail;

	// consume all buffered data
	while (std::uint8_t seq = psmove_poll(controller_handle)) {
		result = poll_result::success;
		last_poll = std::chrono::high_resolution_clock::now();
	}

	return result;
}

poll_result poll(const gem_t::gem_controller& controller)
{
	return move::psmoveapi::poll(controller.psmove_handle.get());
}

void update_rumble(gem_t::gem_controller& controller)
{
	psmove_set_rumble(controller.psmove_handle.get(), controller.rumble);
}

void update_color(const gem_t::gem_controller& controller)
{
	const auto color = controller.sphere_rgb;
	const auto handle = controller.psmove_handle.get();

	psmove_set_leds(handle, color.r * 255, color.g * 255, color.b * 255);
	psmove_update_leds(handle);
}

/*
void psmoveapi_thread::on_task()
{
	while (fxm::check<camera_thread>() && !Emu.IsStopped())
	{
		std::chrono::steady_clock::time_point frame_start = std::chrono::steady_clock::now();

		if (Emu.IsPaused())
		{
			std::this_thread::sleep_for(1ms);
			continue;
		}

		{
			semaphore_lock lock(mutex_poll);

			for (auto controller : m_controllers)
			{
				const auto handle = controller.second;

				move::psmoveapi::poll(handle);
			}
		}

		//std::this_thread::sleep_for(10ms);

		const std::chrono::microseconds frame_target_time{ static_cast<u32>(1000000.0 / 90) };

		std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

		std::chrono::microseconds frame_processing_time = std::chrono::duration_cast<std::chrono::microseconds>(now - frame_start);

		if (frame_processing_time < frame_target_time)
		{
			std::chrono::microseconds frame_idle_time = frame_target_time - frame_processing_time;
			std::this_thread::sleep_for(frame_idle_time);
		}
	}
}
*/

} // namespace psmoveapi

namespace map {

/**
 * \brief Maps Move controller data (digital buttons, and analog Trigger data) to DS3 pad input.
 *        Unavoidably buttons conflict with DS3 mappings, which is problematic for some games.
 * \param port_no DS3 port number to use
 * \param digital_buttons Bitmask filled with CELL_GEM_CTRL_* values
 * \param analog_t Analog value of Move's Trigger. Currently mapped to R2.
 * \return true on success, false if port_no controller is invalid
 */
static bool ds3_input_to_pad(const u32 port_no, be_t<u16>& digital_buttons, be_t<u16>& analog_t)
{
	const auto handler = fxm::get<pad_thread>();

	if (!handler)
	{
		return false;
	}

	const PadInfo& rinfo = handler->GetInfo();

	if (port_no >= rinfo.max_connect || port_no >= rinfo.now_connect)
	{
		return false;
	}

	auto& pads = handler->GetPads();
	auto pad = pads[port_no];

	for (Button& button : pad->m_buttons)
	{
		//here we check btns, and set pad accordingly,
		if (button.m_offset == CELL_PAD_BTN_OFFSET_DIGITAL2)
		{
			if (button.m_pressed) pad->m_digital_2 |= button.m_outKeyCode;
			else pad->m_digital_2 &= ~button.m_outKeyCode;

			switch (button.m_outKeyCode)
			{
			case CELL_PAD_CTRL_SQUARE:
				pad->m_press_square = button.m_value;
				break;
			case CELL_PAD_CTRL_CROSS:
				pad->m_press_cross = button.m_value;
				break;
			case CELL_PAD_CTRL_CIRCLE:
				pad->m_press_circle = button.m_value;
				break;
			case CELL_PAD_CTRL_TRIANGLE:
				pad->m_press_triangle = button.m_value;
				break;
			case CELL_PAD_CTRL_R1:
				pad->m_press_R1 = button.m_value;
				break;
			case CELL_PAD_CTRL_L1:
				pad->m_press_L1 = button.m_value;
				break;
			case CELL_PAD_CTRL_R2:
				pad->m_press_R2 = button.m_value;
				break;
			case CELL_PAD_CTRL_L2:
				pad->m_press_L2 = button.m_value;
				break;
			default: break;
			}
		}

		if (button.m_flush)
		{
			button.m_pressed = false;
			button.m_flush = false;
			button.m_value = 0;
		}
	}

	memset(&digital_buttons, 0, sizeof(digital_buttons));

	// map the Move key to R1 and the Trigger to R2

	if (pad->m_press_R1)
		digital_buttons |= CELL_GEM_CTRL_MOVE;
	if (pad->m_press_R2)
		digital_buttons |= CELL_GEM_CTRL_T;

	if (pad->m_press_cross)
		digital_buttons |= CELL_GEM_CTRL_CROSS;
	if (pad->m_press_circle)
		digital_buttons |= CELL_GEM_CTRL_CIRCLE;
	if (pad->m_press_square)
		digital_buttons |= CELL_GEM_CTRL_SQUARE;
	if (pad->m_press_triangle)
		digital_buttons |= CELL_GEM_CTRL_TRIANGLE;
	if (pad->m_digital_1)
		digital_buttons |= CELL_GEM_CTRL_SELECT;
	if (pad->m_digital_2)
		digital_buttons |= CELL_GEM_CTRL_START;

	analog_t = pad->m_press_R2;

	return true;
}

/**
 * \brief Maps external Move controller data to DS3 input
 *	      Implementation detail: CellGemExtPortData's digital/analog fields map the same way as
 *	      libPad, so no translation is needed.
 * \param port_no DS3 port number to use
 * \param ext External data to modify
 * \return true on success, false if port_no controller is invalid
 */
static bool ds3_input_to_ext(const u32 port_no, CellGemExtPortData& ext)
{
	const auto handler = fxm::get<pad_thread>();

	if (!handler)
	{
		return false;
	}

	auto& pads = handler->GetPads();

	const PadInfo& rinfo = handler->GetInfo();

	if (port_no >= rinfo.max_connect)
	{
		return false;
	}

	//We have a choice here of NO_DEVICE or READ_FAILED...lets try no device for now
	if (port_no >= rinfo.now_connect)
	{
		return false;
	}

	auto pad = pads[port_no];

	ext.status = 0; // CELL_GEM_EXT_CONNECTED | CELL_GEM_EXT_EXT0 | CELL_GEM_EXT_EXT1
	ext.analog_left_x = pad->m_analog_left_x;
	ext.analog_left_y = pad->m_analog_left_y;
	ext.analog_right_x = pad->m_analog_right_x;
	ext.analog_right_y = pad->m_analog_right_y;
	ext.digital1 = pad->m_digital_1;
	ext.digital2 = pad->m_digital_2;

	return true;
}

static bool psmove_input_to_pad(const gem_t::gem_controller& controller, be_t<u16>& digital_buttons, be_t<u16>& analog_t)
{
	const auto handle = controller.psmove_handle.get();
	const auto buttons = psmove_get_buttons(handle);
	const auto trigger = psmove_get_trigger(handle);

	memset(&digital_buttons, 0, sizeof(digital_buttons));

	if (buttons & Btn_MOVE)
		digital_buttons |= CELL_GEM_CTRL_MOVE;
	if (buttons & Btn_T)
		digital_buttons |= CELL_GEM_CTRL_T;

	if (buttons & Btn_CROSS)
		digital_buttons |= CELL_GEM_CTRL_CROSS;
	if (buttons & Btn_CIRCLE)
		digital_buttons |= CELL_GEM_CTRL_CIRCLE;
	if (buttons & Btn_SQUARE)
		digital_buttons |= CELL_GEM_CTRL_SQUARE;
	if (buttons & Btn_TRIANGLE)
		digital_buttons |= CELL_GEM_CTRL_TRIANGLE;
	if (buttons & Btn_SELECT)
		digital_buttons |= CELL_GEM_CTRL_SELECT;
	if (buttons & Btn_START)
		digital_buttons |= CELL_GEM_CTRL_START;

	analog_t = trigger * float(USHRT_MAX) / float(UCHAR_MAX);

	return true;
}
static bool psmove_input_to_gem(const gem_t::gem_controller& controller, vm::ptr<CellGemState>& gem_state)
{
	const auto handle = controller.psmove_handle.get();

	if (!psmove_has_calibration(handle))
	{
		__debugbreak();
	}

	return false;
}

static bool psmove_input_to_inertial(const gem_t::gem_controller& controller, vm::ptr<CellGemInertialState>& inertial_state)
{
	const auto handle = controller.psmove_handle.get();

	if (!psmove_has_calibration(handle))
	{
		__debugbreak();
	}

	float ax, ay, az;
	psmove_get_accelerometer_frame(handle, Frame_SecondHalf, &ax, &ay, &az);
	auto* ac = inertial_state->accelerometer;
	ac[0] = ax;
	ac[1] = ay;
	ac[2] = az;
	ac[3] = 0;

	float gx, gy, gz;
	psmove_get_gyroscope_frame(handle, Frame_SecondHalf, &gx, &gy, &gz);
	auto* gr = inertial_state->gyro;
	gr[0] = gx;
	gr[1] = gy;
	gr[2] = gz;
	gr[3] = 0;

	auto ac_b = inertial_state->accelerometer_bias;
	ac_b[0] = ac_b[1] = ac_b[2] = ac_b[3] = 0;

	auto gr_b = inertial_state->gyro_bias;
	gr_b[0] = gr_b[1] = gr_b[2] = gr_b[3] = 0;

	return false;
}

} // namespace map
} // namespace move

// *********************
// * cellGem functions *
// *********************

s32 cellGemCalibrate(u32 gem_num)
{
	cellGem.todo("cellGemCalibrate(gem_num=%d)", gem_num);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	// if (g_cfg.io.move == move_handler::fake)
	{
		gem->controllers[gem_num].calibrated_magnetometer = true;
		gem->status_flags = CELL_GEM_FLAG_CALIBRATION_OCCURRED | CELL_GEM_FLAG_CALIBRATION_SUCCEEDED;
	}

	return CELL_OK;
}

s32 cellGemClearStatusFlags(u32 gem_num, u64 mask)
{
	cellGem.todo("cellGemClearStatusFlags(gem_num=%d, mask=0x%x)", gem_num, mask);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	gem->status_flags &= ~mask;

	return CELL_OK;
}

s32 cellGemConvertVideoFinish()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemConvertVideoStart()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemEnableCameraPitchAngleCorrection(u32 enable_flag)
{
	cellGem.todo("cellGemEnableCameraPitchAngleCorrection(enable_flag=%d", enable_flag);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	gem->enable_pitch_correction = !!enable_flag;

	return CELL_OK;
}

s32 cellGemEnableMagnetometer(u32 gem_num, u32 enable)
{
	cellGem.todo("cellGemEnableMagnetometer(gem_num=%d, enable=0x%x)", gem_num, enable);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!gem->is_controller_ready(gem_num))
	{
		return CELL_GEM_NOT_CONNECTED;
	}

	gem->controllers[gem_num].enabled_magnetometer = !!enable;

	return CELL_OK;
}

s32 cellGemEnd()
{
	cellGem.warning("cellGemEnd()");

	if (!fxm::remove<gem_t>())
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	return CELL_OK;
}

s32 cellGemFilterState(u32 gem_num, u32 enable)
{
	cellGem.warning("cellGemFilterState(gem_num=%d, enable=%d)", gem_num, enable);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	gem->controllers[gem_num].enabled_filtering = !!enable;

	return CELL_OK;
}

s32 cellGemForceRGB(u32 gem_num, float r, float g, float b)
{
	cellGem.todo("cellGemForceRGB(gem_num=%d, r=%f, g=%f, b=%f)", gem_num, r, g, b);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	auto& controller = gem->controllers[gem_num];
	controller.sphere_rgb = gem_t::gem_color(r, g, b);

	if (g_cfg.io.move == move_handler::move)
	{
		move::psmoveapi::update_color(controller);
	}

	return CELL_OK;
}

s32 cellGemGetAccelerometerPositionInDevice()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemGetAllTrackableHues(vm::ptr<u8> hues)
{
	cellGem.todo("cellGemGetAllTrackableHues(hues=*0x%x)");
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!hues)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	for (auto i = 0; i < 360; ++i)
	{
		hues[i] = true;
	}

	return CELL_OK;
}

s32 cellGemGetCameraState(vm::ptr<CellGemCameraState> camera_state)
{
	cellGem.todo("cellGemGetCameraState(camera_state=0x%x)", camera_state);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!camera_state)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	camera_state->exposure_time = 1.0f / 60.0f;	// TODO: use correct framerate
	camera_state->gain = 1.0;

	return CELL_OK;
}

s32 cellGemGetEnvironmentLightingColor(vm::ptr<float> r, vm::ptr<float> g, vm::ptr<float> b)
{
	cellGem.todo("cellGemGetEnvironmentLightingColor(r=*0x%x, g=*0x%x, b=*0x%x)", r, g, b);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!r || !g || !b)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	*r = 128;
	*g = 128;
	*b = 128;

	return CELL_OK;
}

s32 cellGemGetHuePixels()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemGetImageState(u32 gem_num, vm::ptr<CellGemImageState> image_state)
{
	cellGem.todo("cellGemGetImageState(gem_num=%d, image_state=&0x%x)", gem_num, image_state);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	if (g_cfg.io.move == move_handler::fake)
	{
		auto shared_data = fxm::get_always<gem_camera_shared>();

		image_state->frame_timestamp = shared_data->frame_timestamp.load();
		image_state->timestamp = image_state->frame_timestamp + 10;   // arbitrarily define 10 usecs of frame processing
		image_state->visible = true;
		image_state->u = 0;
		image_state->v = 0;
		image_state->r = 20;
		image_state->r_valid = true;
		image_state->distance = 2 * 1000;   // 2 meters away from camera
		// TODO
		image_state->projectionx = 1;
		image_state->projectiony = 1;
	}
	else
	{
		auto shared_data = fxm::get_always<gem_camera_shared>();

		image_state->frame_timestamp = shared_data->frame_timestamp.load();
		image_state->timestamp = image_state->frame_timestamp + 10;   // arbitrarily define 10 usecs of frame processing
		image_state->visible = true;
		image_state->u = 0;
		image_state->v = 0;
		image_state->r = 20;
		image_state->r_valid = true;
		image_state->distance = 2 * 1000;   // 2 meters away from camera
											// TODO
		image_state->projectionx = 1;
		image_state->projectiony = 1;
	}

	return CELL_OK;
}

s32 cellGemGetInertialState(u32 gem_num, u32 state_flag, u64 timestamp, vm::ptr<CellGemInertialState> inertial_state)
{
	cellGem.todo("cellGemGetInertialState(gem_num=%d, state_flag=%d, timestamp=0x%x, inertial_state=0x%x)", gem_num, state_flag, timestamp, inertial_state);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num) || !inertial_state || !gem->is_controller_ready(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	// TODO(velocity): Abstract with gem state func
	if (g_cfg.io.move == move_handler::fake)
	{
		move::map::ds3_input_to_pad(gem_num, inertial_state->pad.digitalbuttons, inertial_state->pad.analog_T);
		move::map::ds3_input_to_ext(gem_num, inertial_state->ext);
	}

	if (g_cfg.io.move == move_handler::move)
	{
		auto& handle = gem->controllers[gem_num];
		move::psmoveapi::poll(handle);
		//if (move::psmoveapi::poll(handle) != move::psmoveapi::poll_result::fail)
		{
			move::map::psmove_input_to_pad(handle, inertial_state->pad.digitalbuttons, inertial_state->pad.analog_T);
			move::map::psmove_input_to_inertial(handle, inertial_state);

			// save inertial_state
			handle.buffered_inertial_state = *inertial_state;
		}
		//else
		{
			// polling failed, load saved inertial_state
			// *inertial_state = handle.buffered_inertial_state;
		}
	}

	// TODO: should this be in if above?
	inertial_state->timestamp = gem->timer.GetElapsedTimeInMicroSec();
	inertial_state->counter = gem->inertial_counter++;

	return CELL_OK;
}

s32 cellGemGetInfo(vm::ptr<CellGemInfo> info)
{
	cellGem.todo("cellGemGetInfo(info=*0x%x)", info);

	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!info)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	// TODO: Support connecting PlayStation Move controllers
	info->max_connect = gem->attribute.max_connect;
	info->now_connect = gem->connected_controllers;

	for (int i = 0; i < CELL_GEM_MAX_NUM; i++)
	{
		info->status[i] = gem->controllers[i].status;
		info->port[i] = gem->controllers[i].port;
	}

	return CELL_OK;
}

s32 cellGemGetMemorySize(s32 max_connect)
{
	cellGem.warning("cellGemGetMemorySize(max_connect=%d)", max_connect);

	if (max_connect > CELL_GEM_MAX_NUM || max_connect <= 0)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	return max_connect <= 2 ? 0x120000 : 0x140000;
}

s32 cellGemGetRGB(u32 gem_num, vm::ptr<float> r, vm::ptr<float> g, vm::ptr<float> b)
{
	cellGem.todo("cellGemGetRGB(gem_num=%d, r=*0x%x, g=*0x%x, b=*0x%x)", gem_num, r, g, b);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num) | !r || !g || !b )
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	auto& sphere_color = gem->controllers[gem_num].sphere_rgb;
	*r = sphere_color.r;
	*g = sphere_color.g;
	*b = sphere_color.b;

	return CELL_OK;
}

s32 cellGemGetRumble(u32 gem_num, vm::ptr<u8> rumble)
{
	cellGem.todo("cellGemGetRumble(gem_num=%d, rumble=*0x%x)", gem_num, rumble);
	auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num) || !rumble)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	*rumble = gem->controllers[gem_num].rumble;

	return CELL_OK;
}

s32 cellGemGetState(u32 gem_num, u32 flag, u64 time_parameter, vm::ptr<CellGemState> gem_state)
{
	cellGem.todo("cellGemGetState(gem_num=%d, flag=0x%x, time=0x%llx, gem_state=*0x%x)", gem_num, flag, time_parameter, gem_state);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	if (g_cfg.io.move == move_handler::fake)
	{
		move::map::ds3_input_to_pad(gem_num, gem_state->pad.digitalbuttons, gem_state->pad.analog_T);
		move::map::ds3_input_to_ext(gem_num, gem_state->ext);
	}

	if (g_cfg.io.move == move_handler::move)
	{
		auto& handle = gem->controllers[gem_num];
		move::psmoveapi::poll(handle);
		// if (move::psmoveapi::poll(handle) != move::psmoveapi::poll_result::fail)
		{
			move::map::psmove_input_to_pad(handle, gem_state->pad.digitalbuttons, gem_state->pad.analog_T);
			move::map::psmove_input_to_gem(handle, gem_state);

			// save gem_state
			handle.buffered_gem_state = *gem_state;
			// cellGem.fatal("SUCCESS %d", gem_state->pad.digitalbuttons.value() & CELL_GEM_CTRL_MOVE);
		}
		// else
		{
			// polling failed, load saved gem_state
			// *gem_state = handle.buffered_gem_state;
			// cellGem.fatal("FAILLLL %d", gem_state->pad.digitalbuttons.value() & CELL_GEM_CTRL_MOVE);
		}
	}

	gem_state->tracking_flags = CELL_GEM_TRACKING_FLAG_POSITION_TRACKED |
		CELL_GEM_TRACKING_FLAG_VISIBLE;
	gem_state->timestamp = gem->timer.GetElapsedTimeInMicroSec();

	gem_state->quat[3] = 1.0;

	return CELL_GEM_NOT_CONNECTED;
}

s32 cellGemGetStatusFlags(u32 gem_num, vm::ptr<u64> flags)
{
	cellGem.todo("cellGemGetStatusFlags(gem_num=%d, flags=*0x%x)", gem_num, flags);
	const auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num) || !flags)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	*flags = gem->status_flags;

	return CELL_OK;
}

s32 cellGemGetTrackerHue()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemHSVtoRGB()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemInit(vm::cptr<CellGemAttribute> attribute)
{
	cellGem.warning("cellGemInit(attribute=*0x%x)", attribute);
	const auto gem = fxm::make<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_ALREADY_INITIALIZED;
	}

	if (!attribute || !attribute->spurs_addr || attribute->max_connect > CELL_GEM_MAX_NUM)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	gem->attribute = *attribute;

	if (g_cfg.io.move == move_handler::move)
	{
		// Initialize psmoveapi
		move::psmoveapi::init(gem.get());
	}

	for (auto gem_num = 0; gem_num < CELL_GEM_MAX_NUM; gem_num++)
	{
		gem->reset_controller(gem.get(), gem_num);
	}

	// TODO: is this correct?
	gem->timer.Start();

	return CELL_OK;
}

s32 cellGemInvalidateCalibration()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemIsTrackableHue(u32 hue)
{
	cellGem.todo("cellGemIsTrackableHue(hue=%d)", hue);
	const auto gem = fxm::get<gem_t>();

	if (!gem || hue > 359)
	{
		return false;
	}

	return true;
}

s32 cellGemPrepareCamera()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemPrepareVideoConvert(vm::cptr<CellGemVideoConvertAttribute> vc_attribute)
{
	cellGem.todo("cellGemPrepareVideoConvert(vc_attribute=*0x%x)", vc_attribute);
	auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!vc_attribute)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	const auto vc = *vc_attribute;

	if (!vc_attribute || vc.version == 0 || vc.output_format == 0 ||
		vc.conversion_flags & CELL_GEM_VIDEO_CONVERT_UNK3 && !vc.buffer_memory)
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	if (vc.video_data_out & 0x1f || vc.buffer_memory & 0xff)
	{
		return CELL_GEM_ERROR_INVALID_ALIGNMENT;
	}

	gem->vc_attribute = vc;

	return CELL_OK;
}

s32 cellGemReadExternalPortDeviceInfo()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemReset(u32 gem_num)
{
	cellGem.todo("cellGemReset(gem_num=%d)", gem_num);
	auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	gem->reset_controller(gem.get(), gem_num);

	// TODO: is this correct?
	gem->timer.Start();

	return CELL_OK;
}

s32 cellGemSetRumble(u32 gem_num, u8 rumble)
{
	cellGem.todo("cellGemSetRumble(gem_num=%d, rumble=0x%x)", gem_num, rumble);
	auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	if (!check_gem_num(gem_num))
	{
		return CELL_GEM_ERROR_INVALID_PARAMETER;
	}

	auto& controller = gem->controllers[gem_num];
	controller.rumble = rumble;

	if (g_cfg.io.move == move_handler::move)
	{
		move::psmoveapi::update_rumble(controller);
	}

	return CELL_OK;
}

s32 cellGemSetYaw()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemTrackHues()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemUpdateFinish()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

s32 cellGemUpdateStart(vm::cptr<void> camera_frame, u64 timestamp)
{
	cellGem.todo("cellGemUpdateStart(camera_frame=*0x%x, timestamp=%d)", camera_frame, timestamp);
	auto gem = fxm::get<gem_t>();

	if (!gem)
	{
		return CELL_GEM_ERROR_UNINITIALIZED;
	}

	return CELL_OK;
}

s32 cellGemWriteExternalPort()
{
	UNIMPLEMENTED_FUNC(cellGem);
	return CELL_OK;
}

DECLARE(ppu_module_manager::cellGem)("libgem", []()
{
	REG_FUNC(libgem, cellGemCalibrate);
	REG_FUNC(libgem, cellGemClearStatusFlags);
	REG_FUNC(libgem, cellGemConvertVideoFinish);
	REG_FUNC(libgem, cellGemConvertVideoStart);
	REG_FUNC(libgem, cellGemEnableCameraPitchAngleCorrection);
	REG_FUNC(libgem, cellGemEnableMagnetometer);
	REG_FUNC(libgem, cellGemEnd);
	REG_FUNC(libgem, cellGemFilterState);
	REG_FUNC(libgem, cellGemForceRGB);
	REG_FUNC(libgem, cellGemGetAccelerometerPositionInDevice);
	REG_FUNC(libgem, cellGemGetAllTrackableHues);
	REG_FUNC(libgem, cellGemGetCameraState);
	REG_FUNC(libgem, cellGemGetEnvironmentLightingColor);
	REG_FUNC(libgem, cellGemGetHuePixels);
	REG_FUNC(libgem, cellGemGetImageState);
	REG_FUNC(libgem, cellGemGetInertialState);
	REG_FUNC(libgem, cellGemGetInfo);
	REG_FUNC(libgem, cellGemGetMemorySize);
	REG_FUNC(libgem, cellGemGetRGB);
	REG_FUNC(libgem, cellGemGetRumble);
	REG_FUNC(libgem, cellGemGetState);
	REG_FUNC(libgem, cellGemGetStatusFlags);
	REG_FUNC(libgem, cellGemGetTrackerHue);
	REG_FUNC(libgem, cellGemHSVtoRGB);
	REG_FUNC(libgem, cellGemInit);
	REG_FUNC(libgem, cellGemInvalidateCalibration);
	REG_FUNC(libgem, cellGemIsTrackableHue);
	REG_FUNC(libgem, cellGemPrepareCamera);
	REG_FUNC(libgem, cellGemPrepareVideoConvert);
	REG_FUNC(libgem, cellGemReadExternalPortDeviceInfo);
	REG_FUNC(libgem, cellGemReset);
	REG_FUNC(libgem, cellGemSetRumble);
	REG_FUNC(libgem, cellGemSetYaw);
	REG_FUNC(libgem, cellGemTrackHues);
	REG_FUNC(libgem, cellGemUpdateFinish);
	REG_FUNC(libgem, cellGemUpdateStart);
	REG_FUNC(libgem, cellGemWriteExternalPort);
});
