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

const double mPI = 3.14159265358979323846;
const double mHalfPI = mPI * 0.5;

HairGeo sHair;

using std::cout;
using std::cerr;
using std::endl;
using std::string;
using std::vector;
using std::pair;
using std::to_string;


class MyGLCanvas : public nanogui::GLCanvas {
public:
	Eigen::Vector3f mRotation;
	float mZoom;

	MyGLCanvas(Widget *parent) : nanogui::GLCanvas(parent), mRotation(nanogui::Vector3f(0, 0, 0)), mZoom(1.0f), mDragging(false), mRootColor(nanogui::Color(237,207,180,255)), mTipColor(nanogui::Color(123,0,0,255)) {
		using namespace nanogui;

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

	void setHairColor() {
		sHair.resetIter();

		HairVertex vtx;

		auto numPoints = sHair.numPoints();
		nanogui::MatrixXf colors(3, numPoints);		
		
		for (auto i = 0u; i < numPoints; i++) {
			sHair >> vtx;
			nanogui::Color c = mRootColor * (1 - vtx.t) + mTipColor * vtx.t;
			colors.col(vtx.id) << c.r(), c.g(), c.b();
		}
		
		mShader.bind();
		mShader.uploadAttrib("color", colors);
	}

	void setHair(){
		sHair = HairCreator::createRadialHair(0, 1000, 5, 0.3f);		

		auto numSegments = sHair.numSegments();
		auto numPoints = sHair.numPoints();

		sHair.resetIter();

		using namespace nanogui;
       
		MatrixXu indices(2, numSegments);
		MatrixXf colors(3, numPoints);
		MatrixXf positions(3, numPoints);

		HairSegment segment;
		for (auto segId = 0u; segId < numSegments; segId++) {			
			sHair >> segment;
			indices.col(segId) << segment.a.id, segment.b.id;
		}

		setHairColor();

		Eigen::Vector3f point;
		sHair.resetIter();
		
		for (auto pid = 0u; pid < numPoints; pid++) {
			sHair >> point;
			positions.col(pid) << point(0), point(1), point(2);
		}

        mShader.bind();
        mShader.uploadIndices(indices);
        mShader.uploadAttrib("position", positions);        
    }

    ~MyGLCanvas() {
        mShader.free();
    }

    void setRotation(nanogui::Vector3f vRotation) {
        mRotation = vRotation;
    }

    virtual void drawGL() override {
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
		auto numSegments = sHair.numSegments();
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
private:
    nanogui::GLShader mShader;
	bool mDragging;
	nanogui::Vector2i mClickPoint;
	Eigen::Vector3f mInitRotation;
	nanogui::Color mRootColor;
	nanogui::Color mTipColor;
};


class FloatField : public nanogui::Widget {
public:
	FloatField(nanogui::Widget *parent, const std::string &label, float value, float min, float max) : nanogui::Widget(parent) {
		setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 0, 0));

		new nanogui::Label(this, label);
		auto slider = new nanogui::Slider(this);
		slider->setRange(std::pair<float, float>(min, max));
		slider->setValue(value);
		auto txtBox = new nanogui::TextBox(this);
		slider->setCallback([txtBox](float v) {
			std::stringstream stream;
			stream << std::fixed << std::setprecision(2) << v;
			txtBox->setValue(stream.str());
		});
		(slider->callback())(value);
	}
};

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

		mTabChooser = new ComboBox(tools, { "Solver","Display" });
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

			new FloatField(props, "Timestep", 5.0f, 1.0f, 10.0f);
			new FloatField(props, "Gravity Y", -9.81f, -10.0f, 10.0f);		

			mTabs[0] = props;
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

			mTabs[1] = props;
			tabLay->setAnchor(props, AdvancedGridLayout::Anchor(0, 1, nanogui::Alignment::Fill, nanogui::Alignment::Minimum));
		}

		winLay->setAnchor(tools, AdvancedGridLayout::Anchor(1, 0 , nanogui::Alignment::Fill, nanogui::Alignment::Minimum));

		Widget *playbackPanel = new Widget(window);
		playbackPanel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Minimum, 0, 0));

		new Button(playbackPanel, "Reset");
		new Button(playbackPanel, "Play/Pause");

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
private:
	static const int mNumTabs = 2;
    MyGLCanvas *mCanvas;
	Widget * mTabs[mNumTabs];
	nanogui::ComboBox * mTabChooser;
};

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
