#ifndef C2M_IMGUI_VIEWER_HPP
#define C2M_IMGUI_VIEWER_HPP

#include <crash2mesh/algorithm/mesh_builder.hpp>
#include <crash2mesh/algorithm/mesh_decimater.hpp>
#include <crash2mesh/algorithm/mesh_analyzer.hpp>
#include <crash2mesh/algorithm/surface_extractor.hpp>
#include <crash2mesh/io/erfh5/reader.hpp>
#include <crash2mesh/io/c2m/c2m_writer.hpp>

#include <crash2mesh/viewer/animation_viewer.hpp>
#include <mutex>

// A very good tutorial for imgui:
// https://eliasdaler.github.io/using-imgui-with-sfml-pt1/
// https://eliasdaler.github.io/using-imgui-with-sfml-pt2/

struct ImGuiContext;

namespace c2m {

struct rgb
{
    float r; // in [0, 1]
    float g; // in [0, 1]
    float b; // in [0, 1]
};

struct hsv
{
    float h; // in [0, 360]
    float s; // in [0, 1]
    float v; // in [0, 1]
};

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

        /**
         * Run the viewer.
         */
        void run();

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
		void  reload_font(int font_size = 14);

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
		std::string usage() const;
		
		/*
		 * Additional stuff for decimation
		 */
		virtual bool key_press_event(int key, int modifiers);

		virtual bool mouse_release_event(int x, int y, int button, int modifiers);

		virtual bool mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers);

		bool removeCurrentPartAndModel();
		bool removeAllElse();

		void drawInfoPanel();

		void drawDecimationPanel();

		void drawProcessing();

		bool openDialog();

		bool openFile(const std::string& fileName_);

		static void HelpMarker(const char* desc);

		bool createDrawableEpicenters();

		bool buildParts();

		bool createDrawableParts();

		bool calcEpicenters();

		bool toggleExpandParts();

		bool mergeParts();

		bool createDrawableScene();

		bool decimatePartwise();

		bool decimateScene();

		bool updateFrame();

		bool toggleStrainColors();

		bool updateWireframeVisibility();
		bool updateBoundaryVisibility();
        bool updateVertexVisibility();
        bool updateFaceVisibility();

		bool exportCurrentPart();
		bool exportCurrentPartDialog();
		bool exportScene();
		bool exportSceneDialog();

		easy3d::vec3 getStrainColor(float strain);

		hsv minColorHSV{90.0f/360.0f, 0.0f, 1.0f};
		hsv maxColorHSV{0.0f, 1.0, 1.0f};

		bool fullReload = false;

		bool show_overlay = true;

		bool targetCam = false;

		float maxStrain = 1.0f;

    	std::vector<Part::Ptr> parts;
		std::string fileName;

		MeshDecimater deciParts;
		MeshDecimater deciScene;
		Scene::Ptr scene;

		MatX3 epicenters;
		VecX meanDists;

		int targetVertices = 0;
		int targetFaces = 0;

		int stage = 0;

		bool partsExpanded = false;

		bool drawFaces = true;
		bool drawWireframes = false;
		bool drawVertices = false;
		bool drawBoundaries = false;

		bool strainColors = true;

		bool animating = false;
		int currentFrame = 0;
		int nVisFrames = 1;
		std::vector<uint> visFrames;

		std::map<easy3d::Model*, std::vector<std::vector<easy3d::vec3>>> modelToFrameToVertexbuffer;
		std::map<easy3d::Model*, std::vector<std::vector<easy3d::vec3>>> modelToFrameToColorbuffer;

		// global stats
		int numFrames = 0;

		float maxStrainGlobal = 0.0f;

		bool updateMaxStrains();
		std::vector<float> frame2maxPlasticStrain;
		std::vector<std::map<int, float>> frame2pMaxPlasticStrain;
		bool updateGlobalStats();
		bool updateCurrentPartStats();

		// Global
		int numTriangles = 0;
		int numVertices = 0;
		int numEdges = 0;
		int numComplexEdges = 0;
		int numBoundaryEdges = 0;
		int num1DFE = 0;
		int num2DFE = 0;
		int numNodes = 0;
		int numParts1D = 0;
		int numParts2D = 0;
		int numParts3D = 0;

		std::string exportPartFilename;
		std::string exportSceneFilename;

		enum class Action {
			NONE,
			LOADFILE,
			RELOADFILE,
			CALCEPICENTERS,
			REDRAW,
			DECIMATEPARTS,
			MERGESCENE,
			DECIMATESCENE,
			EXPORTPART,
			EXPORTSCENE
		};

		Action currentAction = Action::NONE;
		int performInFrames = 5;

		// Per part
		std::map<int, int> pNumComplexEdges;
		std::map<int, int> pNumBoundaryEdges;
		std::map<int, int> pNum2DFE;
		std::map<int, int> pNumNodes;
		std::map<int, bool> pIs2DPart;

	private:
    	std::mutex mut;
	};

}

#endif	// _EASY3D_VIEWER_H_
