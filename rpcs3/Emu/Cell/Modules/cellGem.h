#pragma once

#include "gsl.h"
#include "psmove.h"
#include "Utilities/Timer.h"
#include "Utilities/Thread.h"

static const float CELL_GEM_SPHERE_RADIUS_MM = 22.5f;

// Error codes
enum
{
	CELL_GEM_ERROR_RESOURCE_ALLOCATION_FAILED = 0x80121801,
	CELL_GEM_ERROR_ALREADY_INITIALIZED        = 0x80121802,
	CELL_GEM_ERROR_UNINITIALIZED              = 0x80121803,
	CELL_GEM_ERROR_INVALID_PARAMETER          = 0x80121804,
	CELL_GEM_ERROR_INVALID_ALIGNMENT          = 0x80121805,
	CELL_GEM_ERROR_UPDATE_NOT_FINISHED        = 0x80121806,
	CELL_GEM_ERROR_UPDATE_NOT_STARTED         = 0x80121807,
	CELL_GEM_ERROR_CONVERT_NOT_FINISHED       = 0x80121808,
	CELL_GEM_ERROR_CONVERT_NOT_STARTED        = 0x80121809,
	CELL_GEM_ERROR_WRITE_NOT_FINISHED         = 0x8012180A,
	CELL_GEM_ERROR_NOT_A_HUE                  = 0x8012180B,
};

// Runtime statuses
enum
{
	CELL_GEM_NOT_CONNECTED              = 1,
	CELL_GEM_SPHERE_NOT_CALIBRATED      = 2,
	CELL_GEM_SPHERE_CALIBRATING         = 3,
	CELL_GEM_COMPUTING_AVAILABLE_COLORS = 4,
	CELL_GEM_HUE_NOT_SET                = 5,
	CELL_GEM_NO_VIDEO                   = 6,
	CELL_GEM_TIME_OUT_OF_RANGE          = 7,
	CELL_GEM_NOT_CALIBRATED             = 8,
	CELL_GEM_NO_EXTERNAL_PORT_DEVICE    = 9,
};

// General constants
enum
{
	CELL_GEM_CTRL_CIRCLE                                 = 1 << 5,
	CELL_GEM_CTRL_CROSS                                  = 1 << 6,
	CELL_GEM_CTRL_MOVE                                   = 1 << 2,
	CELL_GEM_CTRL_SELECT                                 = 1 << 0,
	CELL_GEM_CTRL_SQUARE                                 = 1 << 7,
	CELL_GEM_CTRL_START                                  = 1 << 3,
	CELL_GEM_CTRL_T                                      = 1 << 1,
	CELL_GEM_CTRL_TRIANGLE                               = 1 << 4,
	CELL_GEM_DONT_CARE_HUE                               = 4 << 24,
	CELL_GEM_DONT_CHANGE_HUE                             = 8 << 24,
	CELL_GEM_DONT_TRACK_HUE                              = 2 << 24,
	CELL_GEM_EXT_CONNECTED                               = 1 << 0,
	CELL_GEM_EXT_EXT0                                    = 1 << 1,
	CELL_GEM_EXT_EXT1                                    = 1 << 2,
	CELL_GEM_EXTERNAL_PORT_DEVICE_INFO_SIZE              = 38,
	CELL_GEM_EXTERNAL_PORT_OUTPUT_SIZE                   = 40,
	CELL_GEM_FLAG_CALIBRATION_FAILED_BRIGHT_LIGHTING     = 1 << 4,
	CELL_GEM_FLAG_CALIBRATION_FAILED_CANT_FIND_SPHERE    = 1 << 2,
	CELL_GEM_FLAG_CALIBRATION_FAILED_MOTION_DETECTED     = 1 << 3,
	CELL_GEM_FLAG_CALIBRATION_OCCURRED                   = 1 << 0,
	CELL_GEM_FLAG_CALIBRATION_SUCCEEDED                  = 1 << 1,
	CELL_GEM_FLAG_CALIBRATION_WARNING_BRIGHT_LIGHTING    = 1 << 6,
	CELL_GEM_FLAG_CALIBRATION_WARNING_MOTION_DETECTED    = 1 << 5,
	CELL_GEM_FLAG_CAMERA_PITCH_ANGLE_CHANGED             = 1 << 9,
	CELL_GEM_FLAG_CURRENT_HUE_CONFLICTS_WITH_ENVIRONMENT = 1 << 13,
	CELL_GEM_FLAG_LIGHTING_CHANGED                       = 1 << 7,
	CELL_GEM_FLAG_VARIABLE_MAGNETIC_FIELD                = 1 << 10,
	CELL_GEM_FLAG_VERY_COLORFUL_ENVIRONMENT              = 1 << 12,
	CELL_GEM_FLAG_WEAK_MAGNETIC_FIELD                    = 1 << 11,
	CELL_GEM_FLAG_WRONG_FIELD_OF_VIEW_SETTING            = 1 << 8,
	CELL_GEM_INERTIAL_STATE_FLAG_LATEST                  = 0,
	CELL_GEM_INERTIAL_STATE_FLAG_NEXT                    = 2,
	CELL_GEM_INERTIAL_STATE_FLAG_PREVIOUS                = 1,
	CELL_GEM_LATENCY_OFFSET                              = -22000,
	CELL_GEM_MAX_CAMERA_EXPOSURE                         = 511,
	CELL_GEM_MAX_NUM                                     = 4,
	CELL_GEM_MIN_CAMERA_EXPOSURE                         = 40,
	CELL_GEM_STATE_FLAG_CURRENT_TIME                     = 0,
	CELL_GEM_STATE_FLAG_LATEST_IMAGE_TIME                = 1,
	CELL_GEM_STATE_FLAG_TIMESTAMP                        = 2,
	CELL_GEM_STATUS_DISCONNECTED                         = 0,
	CELL_GEM_STATUS_READY                                = 1,
	CELL_GEM_TRACKING_FLAG_POSITION_TRACKED              = 1 << 0,
	CELL_GEM_TRACKING_FLAG_VISIBLE                       = 1 << 1,
	CELL_GEM_VERSION                                     = 2,
};

// Video conversion flags
enum
{
	CELL_GEM_VIDEO_CONVERT_UNK1 = 1 << 0,
	CELL_GEM_VIDEO_CONVERT_UNK2 = 1 << 1,
	CELL_GEM_VIDEO_CONVERT_UNK3 = 1 << 2,
	CELL_GEM_VIDEO_CONVERT_UNK4 = 1 << 3,
};

struct CellGemAttribute
{
	be_t<u32> version;
	be_t<u32> max_connect;
	be_t<u32> memory_ptr;
	be_t<u32> spurs_addr;
	u8 spu_priorities[8];
};

struct CellGemCameraState
{
	be_t<s32> exposure;
	be_t<f32> exposure_time;
	be_t<f32> gain;
	be_t<f32> pitch_angle;
	be_t<f32> pitch_angle_estimate;
};

struct CellGemExtPortData
{
	be_t<u16> status;
	be_t<u16> digital1;
	be_t<u16> digital2;
	be_t<u16> analog_right_x;
	be_t<u16> analog_right_y;
	be_t<u16> analog_left_x;
	be_t<u16> analog_left_y;
	u8 custom[5];
};

struct CellGemImageState
{
	be_t<u64> frame_timestamp;
	be_t<u64> timestamp;
	be_t<f32> u;         // horizontal screen position in pixels
	be_t<f32> v;         // vertical screen position in pixels
	be_t<f32> r;         // size of sphere on screen in pixels
	be_t<f32> projectionx;
	be_t<f32> projectiony;
	be_t<f32> distance;
	u8 visible;
	u8 r_valid;
};

struct CellGemPadData
{
	be_t<u16> digitalbuttons;
	be_t<u16> analog_T;
};

struct CellGemInertialState
{
	be_t<f32> accelerometer[4];
	be_t<f32> gyro[4];
	be_t<f32> accelerometer_bias[4];
	be_t<f32> gyro_bias[4];
	CellGemPadData pad;
	CellGemExtPortData ext;
	be_t<u64> timestamp;
	be_t<s32> counter;
	be_t<f32> temperature;
};

struct CellGemInfo
{
	be_t<u32> max_connect;
	be_t<u32> now_connect;
	be_t<u32> status[CELL_GEM_MAX_NUM];
	be_t<u32> port[CELL_GEM_MAX_NUM];
};

struct CellGemState
{
	be_t<f32> pos[4];
	be_t<f32> vel[4];
	be_t<f32> accel[4];
	be_t<f32> quat[4];
	be_t<f32> angvel[4];
	be_t<f32> angaccel[4];
	be_t<f32> handle_pos[4];
	be_t<f32> handle_vel[4];
	be_t<f32> handle_accel[4];
	CellGemPadData pad;
	CellGemExtPortData ext;
	be_t<u64> timestamp;
	be_t<f32> temperature;
	be_t<f32> camera_pitch_angle;
	be_t<u32> tracking_flags;
};

struct CellGemVideoConvertAttribute
{
	be_t<s32> version;
	be_t<s32> output_format;
	be_t<s32> conversion_flags;
	be_t<f32> gain;
	be_t<f32> red_gain;
	be_t<f32> green_gain;
	be_t<f32> blue_gain;
	be_t<u32> buffer_memory;
	be_t<u32> video_data_out;
	u8 alpha;
};

// **********************
// * HLE helper structs *
// **********************

struct gem_t
{
	struct gem_color
	{
		float r, g, b;

		gem_color() : r(0.0f), g(0.0f), b(0.0f) {}
		gem_color(float r_, float g_, float b_)
		{
			r = clamp(r_);
			g = clamp(g_);
			b = clamp(b_);
		}

		float clamp(float f) const
		{
			return std::max(0.0f, std::min(f, 1.0f));
		}
	};

	struct PSMoveDeleter
	{
		void operator()(PSMove* p)
		{
			psmove_disconnect(p);
		}
	};

	struct gem_controller
	{
		u32 status;                     // connection status (CELL_GEM_STATUS_DISCONNECTED or CELL_GEM_STATUS_READY)
		u32 port;                       // assigned port
		bool enabled_magnetometer;      // whether the magnetometer is enabled (probably used for additional rotational precision)
		bool calibrated_magnetometer;   // whether the magnetometer is calibrated
		bool enabled_filtering;         // whether filtering is enabled
		u8 rumble;                      // rumble intensity
		gem_color sphere_rgb;           // RGB color of the sphere LED

		// PSMoveAPI data
		std::unique_ptr<PSMove, PSMoveDeleter> psmove_handle;
		std::string psmove_serial;		// unique identifier (Bluetooth MAC address)

		gem_controller() :
			status(CELL_GEM_STATUS_DISCONNECTED),
			enabled_filtering(false), rumble(0), sphere_rgb() {}
	};

	CellGemAttribute attribute;
	CellGemVideoConvertAttribute vc_attribute;
	u64 status_flags;
	bool enable_pitch_correction;
	u32 inertial_counter;

	std::array<gem_controller, CELL_GEM_MAX_NUM> controllers;
	u32 connected_controllers;

	Timer timer;

	// helper functions
	bool is_controller_ready(u32 gem_num) const
	{
		return controllers[gem_num].status == CELL_GEM_STATUS_READY;
	}

	void reset_controller(gsl::not_null<gem_t*> gem, u32 gem_num);
};

// TODO: For LED updating
/*
class psmoveapi_thread final : public named_thread
{
public:
	psmoveapi_thread();
	~psmoveapi_thread() override = default;

	std::string get_name() const override { return "PSMoveAPI Thread"; }

protected:
	void on_spawn() override;
	void on_exit() override;
	void on_task() override;
public:
	void on_init(const std::shared_ptr<void>& _this) override;
	void on_stop() override;
};
*/