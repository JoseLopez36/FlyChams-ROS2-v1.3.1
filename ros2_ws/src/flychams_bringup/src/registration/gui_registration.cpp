#include "flychams_bringup/registration/gui_registration.hpp"

using namespace flychams::core;

namespace flychams::bringup
{
	// ════════════════════════════════════════════════════════════════════════════
	// CONSTRUCTOR: Constructor and destructor
	// ════════════════════════════════════════════════════════════════════════════

	void GuiRegistration::onInit()
	{
		// Get windows IDs from parameter server
		fixed_window_ids_ = RosUtils::getParameter<IDs>(node_, "fixed_window_ids");
		dynamic_window_ids_ = RosUtils::getParameter<IDs>(node_, "dynamic_window_ids");

		// Get window count
		int num_fixed_windows = static_cast<int>(fixed_window_ids_.size());
		int num_dynamic_windows = static_cast<int>(dynamic_window_ids_.size());

		// Set fixed window parameters (empty for now)
		IDs fixed_vehicle_ids(num_fixed_windows);
		IDs fixed_camera_ids(num_fixed_windows);
		std::vector<int> fixed_crop_x(num_fixed_windows);
		std::vector<int> fixed_crop_y(num_fixed_windows);
		std::vector<int> fixed_crop_w(num_fixed_windows);
		std::vector<int> fixed_crop_h(num_fixed_windows);
		for (int i = 0; i < num_fixed_windows; i++)
		{
			fixed_vehicle_ids[i] = "";
			fixed_camera_ids[i] = "";
			fixed_crop_x[i] = 0;
			fixed_crop_y[i] = 0;
			fixed_crop_w[i] = 0;
			fixed_crop_h[i] = 0;
		}

		// Set dynamic window parameters (empty for now)
		IDs dynamic_vehicle_ids(num_dynamic_windows);
		IDs dynamic_camera_ids(num_dynamic_windows);
		std::vector<int> dynamic_crop_x(num_dynamic_windows);
		std::vector<int> dynamic_crop_y(num_dynamic_windows);
		std::vector<int> dynamic_crop_w(num_dynamic_windows);
		std::vector<int> dynamic_crop_h(num_dynamic_windows);
		for (int i = 0; i < num_dynamic_windows; i++)
		{
			dynamic_vehicle_ids[i] = "";
			dynamic_camera_ids[i] = "";
			dynamic_crop_x[i] = 0;
			dynamic_crop_y[i] = 0;
			dynamic_crop_w[i] = 0;
			dynamic_crop_h[i] = 0;
		}

		// Send command to set the window images
		ext_tools_->setWindowImageGroup(fixed_window_ids_, fixed_vehicle_ids, fixed_camera_ids, fixed_crop_x, fixed_crop_y, fixed_crop_w, fixed_crop_h);
		ext_tools_->setWindowImageGroup(dynamic_window_ids_, dynamic_vehicle_ids, dynamic_camera_ids, dynamic_crop_x, dynamic_crop_y, dynamic_crop_w, dynamic_crop_h);
	}

	void GuiRegistration::onShutdown()
	{
		// Destroy window IDs
		fixed_window_ids_.clear();
		dynamic_window_ids_.clear();
	}

} // namespace flychams::bringup