#ifndef C2M_IMGUI_VIEWER_HPP
#define C2M_IMGUI_VIEWER_HPP

#include <crash2mesh/algorithm/mesh_builder.hpp>
#include <crash2mesh/algorithm/mesh_decimater.hpp>
#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/surface_extractor.hpp>
#include <crash2mesh/io/erfh5/reader.hpp>
#include <crash2mesh/io/c2m/c2m_writer.hpp>

#include <crash2mesh/viewer/animation_viewer.hpp>

// A very good tutorial for imgui:
// https://eliasdaler.github.io/using-imgui-with-sfml-pt1/
// https://eliasdaler.github.io/using-imgui-with-sfml-pt2/

struct ImGuiContext;

namespace c2m {

    class ImGuiViewer : public AnimationViewer
	{
	public:
        ImGuiViewer(
            const std::string& title = "Easy3D ImGui Viewer",
			int samples = 4,
			int gl_major = 3,
			int gl_minor = 2,
			bool full_screen = false,
			bool resizable = true,
			int depth_bits = 24,
			int stencil_bits = 8
		);

	protected:

		// imgui plugins
		void init() override;

		// draw the widgets
		void pre_draw() override;

		//  the widgets
		void post_draw() override;

		void cleanup() override;

		void post_resize(int w, int h) override;

		bool callback_event_cursor_pos(double x, double y) override;
		bool callback_event_mouse_button(int button, int action, int modifiers) override;
		bool callback_event_keyboard(int key, int action, int modifiers) override;
		bool callback_event_character(unsigned int codepoint) override;
		bool callback_event_scroll(double dx, double dy) override;

		bool mouse_press_event(int x, int y, int button, int modifiers) override;
		
        void draw_menu_file();
        void draw_menu_view();

	protected:
        // Ratio between the framebuffer size and the window size.
        // May be different from the DPI scaling!
        double pixel_ratio();

        double widget_scaling() { return dpi_scaling() / pixel_ratio(); }

		// We don't need a per-window font. So this function is static
		void  reload_font(int font_size = 16);

        // To provide real-time feedback to the user, e.g., current state of the
        // model and viewer, etc. This is implemented as a simple static window
        // with no decoration + a context-menu to choose its position.
        void  draw_overlay(bool* visible);

	protected:
		// Single global context by default, but can be overridden by the user
		static ImGuiContext *	context_;

        // Global variables for all the windows
        float	alpha_;
        bool	movable_;
		
		float   menu_height_;

	protected:
		/*
		 * Additional stuff for decimation
		 */

		void drawInfoPanel();

		void drawDecimationPanel();

		bool openDialog();

		bool openFile(const std::string& fileName_);

		static void HelpMarker(const char* desc);

		bool buildParts();

		bool createDrawableParts();

		bool calcEpicenters();

		bool toggleExpandParts();

		bool mergeParts();
		
		bool createDrawableScene();

		bool decimatePartwise();

		bool toggleExpandScene();

		bool decimateScene();

		bool updateFrame();

		virtual bool key_press_event(int key, int modifiers);

    	std::vector<Part::Ptr> parts;
		std::string fileName;

		MeshDecimater deciParts;
		MeshDecimater deciScene;
		Scene::Ptr scene;

		int targetVertices = 0;
		int targetFaces = 0;

		int stage = -1;

		bool partsExpanded = false;

		int currentFrame = -1;
		// global stats
		int numFrames = 0;

		bool updateGlobalStats();
		int numTriangles = 0;
		int numVertices = 0;

	};

}

#endif	// _EASY3D_VIEWER_H_
