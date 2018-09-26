/*
    src/example4.cpp -- C++ version of an example application that shows
    how to use the OpenGL widget. For a Python implementation, see
    '../python/example4.py'.

    NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <nanogui/button.h>
#include <nanogui/checkbox.h>
#include <nanogui/colorwheel.h>
#include <nanogui/combobox.h>
#include <nanogui/entypo.h>
#include <nanogui/formhelper.h>
#include <nanogui/glcanvas.h>
#include <nanogui/glutil.h>
#include <nanogui/graph.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/label.h>
#include <nanogui/layout.h>
#include <nanogui/messagedialog.h>
#include <nanogui/toolbutton.h>
#include <nanogui/opengl.h>
#include <nanogui/popupbutton.h>
#include <nanogui/progressbar.h>
#include <nanogui/screen.h>
#include <nanogui/slider.h>
#include <nanogui/tabwidget.h>
#include <nanogui/textbox.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/window.h>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>

// Includes for the GLTexture class.
#include <cstdint>
#include <memory>
#include <utility>
#include <iomanip>

#if defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
#if defined(_WIN32)
#  pragma warning(push)
#  pragma warning(disable: 4457 4456 4005 4312)
#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#if defined(_WIN32)
#  pragma warning(pop)
#endif
#if defined(_WIN32)
#  if defined(APIENTRY)
#    undef APIENTRY
#  endif
#  include <windows.h>
#endif

#include "HairSolver/HairGeo.h"
#include "HairSolver/HairCreator.h"
#include "HairSolver/HairSolver.h"

const double mPI = 3.14159265358979323846;
const double mHalfPI = mPI * 0.5;

std::atomic<bool> sRunning{ false };
std::atomic<bool> sSimulationDirty{ false };

HairDoF_Points sHairPoints;
HairModel_FollowTheLeader sHairModel;

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::to_string;

class LogInfo {
public:
	LogInfo() : simulationTime(0), processingTime(0), frame(0){}

	float simulationTime;
	float processingTime;
	unsigned int frame;

};

std::vector<LogInfo> sSimulationLog;

class MyGLCanvas;
void staticSetHair(MyGLCanvas *);
void run(MyGLCanvas * app, std::atomic<bool>& simDirty, std::atomic<bool>& program_is_running, unsigned int update_interval_millisecs);

class MyGLCanvas : public nanogui::GLCanvas {
public:
	std::thread simulationThread;

	HairGeo mHair;
	Eigen::Vector3f mRotation;
	float mZoom;
	unsigned int mNumStrands;
	unsigned int mPointsPerStrand;

	MyGLCanvas(Widget *parent) : nanogui::GLCanvas(parent), mRotation(nanogui::Vector3f(0, 0, 0)), mZoom(1.0f), mDragging(false), mRootColor(nanogui::Color(237,207,180,255)), mTipColor(nanogui::Color(123,0,0,255)), mStepsPerSecLabel(nullptr), mNumStrands(1000), mPointsPerStrand(10){

		mShader.init(
			/* An identifying name */
			"a_simple_shader",

			/* Vertex shader */
			"#version 330\n"
			"uniform mat4 modelViewProj;\n"
			"in vec3 position;\n"
			"in vec3 color;\n"
			"out vec4 frag_color;\n"
			"void main() {\n"
			"    frag_color = vec4(color, 1.0);\n"
			"    gl_Position = modelViewProj * vec4(position, 1.0);\n"
			"}",

			/* Fragment shader */
			"#version 330\n"
			"out vec4 color;\n"
			"in vec4 frag_color;\n"
			"void main() {\n"
			"    color = frag_color;\n"
			"}"
		);

		setHair();
	}
	void setHairPositions(HairDoF_Points &hair) {
		auto numPts = hair.mDof.size()/3;
		nanogui::MatrixXf positions(3, numPts);

		for (auto i = 0u; i < numPts; i++)
			positions.col(i) << hair.mDof[i * 3], hair.mDof[i * 3 + 1], hair.mDof[i * 3 + 2];

		mShader.bind();
		mShader.uploadAttrib("position", positions);
		sSimulationDirty = false;
	}

	void setHairPositions(HairGeo &hair) {
		auto numPoints = hair.numPoints();
		nanogui::MatrixXf positions(3, numPoints);

		Eigen::Vector3f point;
		hair.resetIter();

		for (auto pid = 0u; pid < numPoints; pid++) {
			hair >> point;
			positions.col(pid) << point(0), point(1), point(2);
		}
		mShader.bind();
		mShader.uploadAttrib("position", positions);
	}

	void setHairColor() {
		mHair.resetIter();

		HairVertex vtx;

		auto numPoints = mHair.numPoints();
		nanogui::MatrixXf colors(3, numPoints);		
		
		for (auto i = 0u; i < numPoints; i++) {
			mHair >> vtx;
			nanogui::Color c = mRootColor * (1 - vtx.t) + mTipColor * vtx.t;
			colors.col(vtx.id) << c.r(), c.g(), c.b();
		}
		
		mShader.bind();
		mShader.uploadAttrib("color", colors);
	}

	void setHair() {
		bool isSimulating = simulationThread.joinable();
		if (isSimulating) RunOrPauseSimulation();

		mHair.clear();
		mHair = HairCreator::createRadialHair(0, mNumStrands, mPointsPerStrand, (mPointsPerStrand -1) * sHairModel.mSegmentLength);
		auto numSegments = mHair.numSegments();

		mHair.resetIter();

		nanogui::MatrixXu indices(2, numSegments);

		HairSegment segment;
		for (auto segId = 0u; segId < numSegments; segId++) {			
			mHair >> segment;
			indices.col(segId) << segment.a.id, segment.b.id;
		}
		mShader.bind();
		mShader.uploadIndices(indices);
		setHairColor();
		resetSolver();

		if (isSimulating) RunOrPauseSimulation();
    }

    ~MyGLCanvas() {
        mShader.free();
    }

    void setRotation(nanogui::Vector3f vRotation) {
        mRotation = vRotation;
    }

	void RunOrPauseSimulation() {
		if (!simulationThread.joinable()) {
			sRunning = true;
			simulationThread = std::thread([this]() {run(std::ref(this), std::ref(sSimulationDirty), std::ref(sRunning), 17); });
			//t.detach();
		}
		else {
			sRunning = false;
			simulationThread.join();
		}
	}

	void step(std::atomic<bool>& simDirty) {
		LARGE_INTEGER frequency;        // ticks per second
		LARGE_INTEGER t1, t2;           // ticks
		double elapsedTime;
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&t1);

		sHairModel.step(sHairPoints);

		QueryPerformanceCounter(&t2);
		float processingTime = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;

		if (sSimulationLog.size() == 0) sSimulationLog.push_back(LogInfo());
		LogInfo &prevInfo = sSimulationLog[sSimulationLog.size() - 1];
		LogInfo info;
		info.frame = prevInfo.frame + 1;
		info.processingTime = processingTime;
		info.simulationTime = prevInfo.simulationTime + sHairModel.mTimestep;

		sSimulationLog.push_back(info);
		simDirty = true;
	}

    virtual void drawGL() override {
		if (sSimulationDirty) {
			setHairPositions(sHairPoints);
		}

		if (sSimulationLog.size() == 0) sSimulationLog.push_back(LogInfo());

		/*if (mStepsPerSecLabel) {
			float stepsPerFrame = 0;
			if (sSimulationLog.size() > 1) {
				LogInfo &lastLog = sSimulationLog[sSimulationLog.size() - 1];
				stepsPerFrame = 17.0f / lastLog.processingTime;
			}
			std::stringstream stream;
			stream << std::fixed << std::setprecision(1) << stepsPerFrame;
			mStepsPerSecLabel->setCaption(stream.str());
		}*/
		if (mSimtPerStepLabel) {
			LogInfo &lastLog = sSimulationLog[sSimulationLog.size() - 1];
			std::stringstream stream;
			stream << std::fixed << std::setprecision(1) << lastLog.processingTime;
			mSimtPerStepLabel->setCaption(stream.str());
		}
		if (mStepsLabel) {
			LogInfo &lastLog = sSimulationLog[sSimulationLog.size() - 1];
			mStepsLabel->setCaption(std::to_string(lastLog.frame));
		}

        using namespace nanogui;

        mShader.bind();

        Matrix4f mvp;
        mvp.setIdentity();
		float fTime = 1;// (float)glfwGetTime();
        mvp.topLeftCorner<3,3>() = Eigen::Matrix3f(Eigen::AngleAxisf(mRotation[0]*fTime, Vector3f::UnitX()) *
                                                   Eigen::AngleAxisf(mRotation[1]*fTime,  Vector3f::UnitY()) *
                                                   Eigen::AngleAxisf(mRotation[2]*fTime, Vector3f::UnitZ())) * mZoom;
        mShader.setUniform("modelViewProj", mvp);
        glEnable(GL_DEPTH_TEST);
        /* Draw numSegments segments starting at index 0 */
		auto numSegments = mHair.numSegments();
        mShader.drawIndexed(GL_LINES, 0, numSegments);
        glDisable(GL_DEPTH_TEST);
    }

	virtual bool mouseButtonEvent(const nanogui::Vector2i &p, int button, bool down, int modifiers) override {
		if (!down) mDragging = false;

		if (nanogui::GLCanvas::mouseButtonEvent(p, button, down, modifiers)) return true;

		if (button == GLFW_MOUSE_BUTTON_1) {
			if (down) {
				mClickPoint = p;
				mInitRotation = mRotation;
				mDragging = true;
			}
			return true;
		}
		return false;
	}

	virtual bool mouseMotionEvent(const nanogui::Vector2i &p, const nanogui::Vector2i &rel, int button, int modifiers) override {
		if (mDragging) {
			float sensibility = 0.02f;
			auto diff = p - mClickPoint;
			Eigen::Vector3f delta(-sensibility * (float)diff.y(), -sensibility * (float)diff.x(), 0);
			mRotation = mInitRotation + delta;

			if (mRotation.x() > (float)mHalfPI) mRotation[0] = (float)mHalfPI;
			else if (mRotation.x() < -(float)mHalfPI) mRotation[0] = -(float)mHalfPI;

			return true;
		}
		return false;
	}

	virtual bool scrollEvent(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) override {
		if (nanogui::GLCanvas::scrollEvent(p, rel)) return true;
		mZoom *= (1 + rel.y()*0.1f);
		return true;
	}

	void setRootColor(const nanogui::Color &c) {
		mRootColor = c;
		setHairColor();
	}
	const nanogui::Color & getRootColor() const {
		return mRootColor;
	}
	void setTipColor(const nanogui::Color &c) {
		mTipColor = c;
		setHairColor();
	}
	const nanogui::Color & getTipColor() const {
		return mTipColor;
	}

	void resetSolver() {
		mHair.resetIter();
		sHairPoints = mHair;
		sHairPoints.mDofPrev = sHairPoints.mDof;

		setHairPositions(sHairPoints);
		
		sSimulationLog.clear();
		sSimulationLog.push_back(LogInfo());
	}

	void setStepsPerSecLabel(nanogui::Label *l) {
		mStepsPerSecLabel = l;
	}
	void setSimtimePerStepLabel(nanogui::Label *l) {
		mSimtPerStepLabel = l;
	}
	void setStepsLabel(nanogui::Label *l) {
		mStepsLabel = l;
	}
private:
    nanogui::GLShader mShader;
	bool mDragging;
	nanogui::Vector2i mClickPoint;
	Eigen::Vector3f mInitRotation;
	nanogui::Color mRootColor;
	nanogui::Color mTipColor;
	nanogui::Label *mStepsPerSecLabel, *mStepsLabel, *mSimtPerStepLabel;
};

void staticSetHair(MyGLCanvas *c) {
	c->setHair();
}

template <typename T>
class TypeField : public nanogui::Widget {
public:
	TypeField(nanogui::Widget *parent, const std::string &label, T *attribute, T min, T max, T scale, MyGLCanvas* canvas = nullptr, void (*callable) (MyGLCanvas *c) = 0) : nanogui::Widget(parent) {
		setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 0));

		new nanogui::Label(this, label);
		auto slider = new nanogui::Slider(this);
		slider->setRange(std::pair<T, T>(min, max));
		slider->setValue(*attribute / scale);
		auto txtBox = new nanogui::TextBox(this);
		slider->setCallback([txtBox, attribute, scale, canvas, callable](T v) {
			std::stringstream stream;
			stream << std::fixed << std::setprecision(2) << v;
			txtBox->setValue(stream.str());
			*attribute = v * scale;
			if (callable) callable(canvas);

		});
		(slider->callback())((float)(*attribute / scale)); 		
	}
};

typedef TypeField<float> FloatField;
typedef TypeField<unsigned int> UintField;

class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication() : nanogui::Screen(Eigen::Vector2i(1200, 1000), "Vladimir - Simulation Demo", false) {
        using namespace nanogui;

		auto winLay = new AdvancedGridLayout();
		
        Window *window = new Window(this, "Viewport");
        window->setPosition(Vector2i(15, 15));		
		//window->setLayout(new GroupLayout());
		window->setLayout(winLay);
		
		winLay->appendCol(0, 1);
		winLay->appendCol(0, 1);
		winLay->appendRow(0, 1);
		winLay->appendCol(0, 1);
		winLay->appendCol(0, 1);
		winLay->appendRow(0, 1);

        mCanvas = new MyGLCanvas(window);
        mCanvas->setBackgroundColor({100, 100, 100, 255});
        mCanvas->setSize({ 800, 800 });
		
		winLay->setAnchor(mCanvas, AdvancedGridLayout::Anchor(0, 0));// , 0, 0, Alignment::Middle, Alignment::Middle));

		auto tabLay = new AdvancedGridLayout();
		tabLay->appendCol(0, 1);
		tabLay->appendRow(0, 1);
		tabLay->appendRow(0, 1);
	
        Widget *tools = new Widget(window);
		tools->setLayout(tabLay);// new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 0));

		mTabChooser = new ComboBox(tools, { "Hair", "Solver","Display" });
		//mTabChooser->setFixedHeight(20);
		mTabChooser->setCallback(
			[this](int i) {
				for (auto &tab : this->mTabs) {
					tab->setVisible(false);
					tab->setEnabled(false);
				}
				this->mTabs[this->mTabChooser->selectedIndex()]->setVisible(true);
				this->mTabs[this->mTabChooser->selectedIndex()]->setEnabled(true);				
			}
		);

		tabLay->setAnchor(mTabChooser, AdvancedGridLayout::Anchor(0, 0, nanogui::Alignment::Fill, nanogui::Alignment::Minimum));

		{
			Widget *props = new Widget(tools);
			props->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 0));

			new UintField(props, "N Hairs (x1000)", &mCanvas->mNumStrands, 1, 100, 1000, mCanvas, staticSetHair);
			new UintField(props, "Points per Hair", &mCanvas->mPointsPerStrand, 2, 100, 1, mCanvas, staticSetHair);

			mTabs[0] = props;
			tabLay->setAnchor(props, AdvancedGridLayout::Anchor(0, 1, nanogui::Alignment::Fill, nanogui::Alignment::Minimum));
		}

		{
			Widget *props = new Widget(tools);
			props->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 0));

			new FloatField(props, "Timestep", &sHairModel.mTimestep, 1.0f, 40.0f, 0.001f);//ms
			new FloatField(props, "Gravity Y", &sHairModel.mGravity, -10.0f, 10.0f, 1.0f);// m/ss
			new FloatField(props, "Segment L", &sHairModel.mSegmentLength, 1.0f, 10.0f, 0.01f);//cm

			mTabs[1] = props;
			tabLay->setAnchor(props, AdvancedGridLayout::Anchor(0, 1, nanogui::Alignment::Fill, nanogui::Alignment::Minimum));
		}
		

		{
			Widget *props = new Widget(tools);
			props->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 0, 0));

			new Label(props, "Background");
			(new nanogui::ColorWheel(props, mCanvas->backgroundColor()))->setCallback([this](nanogui::Color c) { mCanvas->setBackgroundColor(c); });
			
			new Label(props, "Root");
			(new nanogui::ColorWheel(props, mCanvas->getRootColor()))->setCallback([this](nanogui::Color c) { mCanvas->setRootColor(c); });

			new Label(props, "Tip");
			(new nanogui::ColorWheel(props, mCanvas->getTipColor()))->setCallback([this](nanogui::Color c) { mCanvas->setTipColor(c); });

			mTabs[2] = props;
			tabLay->setAnchor(props, AdvancedGridLayout::Anchor(0, 1, nanogui::Alignment::Fill, nanogui::Alignment::Minimum));
		}

		winLay->setAnchor(tools, AdvancedGridLayout::Anchor(1, 0 , nanogui::Alignment::Fill, nanogui::Alignment::Minimum));

		Widget *playbackPanel = new Widget(window);
		playbackPanel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Minimum, 0, 0));

		(new Button(playbackPanel, "Reset"))->setCallback([this]() {this->resetSolver();});
		(new Button(playbackPanel, "Play/Pause"))->setCallback([this]() {this->RunOrPauseSimulation(); });
		(new Button(playbackPanel, "Step"))->setCallback([this]() {if (!sRunning) this->step(sSimulationDirty); });
		/*new Label(playbackPanel, "Steps/frame:");
		mCanvas->setStepsPerSecLabel(new Label(playbackPanel, "0.0    "));*/
		new Label(playbackPanel, "Simulation time/step:");
		mCanvas->setSimtimePerStepLabel(new Label(playbackPanel, "0.0    "));
		new Label(playbackPanel, "Step:");
		mCanvas->setStepsLabel(new Label(playbackPanel, "0      "));

		winLay->setAnchor(playbackPanel, AdvancedGridLayout::Anchor(0, 1, 2, 1,nanogui::Alignment::Fill, nanogui::Alignment::Fill));
		 
        performLayout();		

		(mTabChooser->callback())(0);
		//mTabs[1]->setVisible(false);
    }


    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) {
        if (Screen::keyboardEvent(key, scancode, action, modifiers))
            return true;
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            setVisible(false);
            return true;
        }
        return false;
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }

	void RunOrPauseSimulation() {
		mCanvas->RunOrPauseSimulation();
	}

	void step(std::atomic<bool> &simDirty) {
		mCanvas->step(simDirty);
	}

	void drawGL() { mCanvas->drawGL(); }
	void resetSolver() { mCanvas->resetSolver(); }

private:
	static const int mNumTabs = 3;
    MyGLCanvas *mCanvas;
	Widget * mTabs[mNumTabs];
	nanogui::ComboBox * mTabChooser;
};

void run(MyGLCanvas * app, std::atomic<bool>& simDirty, std::atomic<bool>& program_is_running, unsigned int update_interval_millisecs) {
	const auto wait_duration = std::chrono::milliseconds(update_interval_millisecs);
	while (program_is_running) {
		app->step(simDirty);
		std::this_thread::sleep_for(wait_duration);
	}
}

int main(int /* argc */, char ** /* argv */) {
    try {
        nanogui::init();

        /* scoped variables */ {
            nanogui::ref<ExampleApplication> app = new ExampleApplication();
            app->drawAll();
            app->setVisible(true);
            nanogui::mainloop();
        }

        nanogui::shutdown();
    } catch (const std::runtime_error &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
        #if defined(_WIN32)
            MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
        #else
            std::cerr << error_msg << endl;
        #endif
        return -1;
    }

    return 0;
}
