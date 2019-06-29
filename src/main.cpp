#include <igl/readPLY.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/writePLY.h>
#include <igl/file_exists.h>
#include <imgui/imgui.h>
#include <iostream>
#include <random>
#include "mytools.h"
#include <Eigen/SVD>

#include <unistd.h>


class MyContext
{
public:

	std::string sourceFile1 = "../data/synthetic.ply";
	//std::string sourceFile1 = "../result/bun_000_045_315_top3_270_090_top2_180_ear_back_chin.ply";
	std::string sourceFile2 = "../data/bun04.ply";

	MyContext() :GaussianNoise(0.0), point_size(8), line_width(0.5), subsampleRate(1.00), mode(0)
	{
	
		//initial vertices and faces
		if (!igl::file_exists(sourceFile1) )//|| !igl::file_exists(sourceFile2))
		{
			std::cout << "[error] cannot locate model file at '../data/XXX.ply' \nPress any key to exit\n";
			char c;
			std::cin >> c;
			exit(1);
		}
		/*
        //XCODE
		igl::readPLY("../../data/bun000.ply", m1_V, m1_F);
        igl::readPLY("../../data/bun045.ply", m2_V, m2_F);
        */

        std::cout<<"eigen version.:"<<EIGEN_WORLD_VERSION<<","<<EIGEN_MAJOR_VERSION << EIGEN_MINOR_VERSION<<"\n";

        char *path=NULL;
        size_t size;
        path=getcwd(path,size);
        std::cout<<"\n current Path"<<path <<"\n";

        //CLION
        igl::readPLY(sourceFile1, m1_V, m1_F);
		igl::readPLY(sourceFile2, m2_V, m2_F);


	}
	~MyContext() {}
    
    Eigen::MatrixXd my_axis;

	Eigen::MatrixXd m1_V;
	Eigen::MatrixXi m1_F;
	Eigen::MatrixXd m2_V;
	Eigen::MatrixXi m2_F;
    

    float xAngle = 0;
    float yAngle = 0;
    float zAngle = 0;
    float xT = 0;
    float yT = 0;
    float zT = 0;


	float GaussianNoise;
	float point_size;
	float line_width;
    float subsampleRate;
	int mode;

	void reset_display(igl::opengl::glfw::Viewer& viewer)
	{
		static std::default_random_engine generator;
		viewer.data().clear();

		if (mode == 1)
		{
            // apply ICP point to point
			ICP_p2Point( m2_V, m1_V);
		}
		else if(mode == 2){
            // apply subsampling
            if (subsampleRate != 1) {
                int numberOfSamples = round(m2_V.rows()*subsampleRate);
                subsampling(m2_V, numberOfSamples);
            }
		}
		else if(mode == 3){
            // apply Gaussian noise
			addGaussNoise(m2_V, GaussianNoise);
		}
        else if(mode == 4){
            // reset
			xAngle = 0;
			yAngle = 0;
			zAngle = 0;
			xT = 0;
			yT = 0;
			zT = 0;

            igl::readPLY(sourceFile1, m1_V, m1_F);
            igl::readPLY(sourceFile2, m2_V, m2_F);
        }
        else if(mode == 5){
            // WRITE INTO FILE
            Eigen::MatrixXd V(m1_V.rows()+m2_V.rows(),3);
            V <<m1_V,m2_V;
            Eigen::MatrixXi F(m1_F.rows()+m2_F.rows(),m1_F.cols());
            F<<m1_F,(m2_F.array()+m1_F.rows());
            std::string filename = "my_bun.ply";
            igl::writePLY(filename, V, F);
        }
		else if(mode == 6){
            // apply ICP point to plane
			ICP_p2Plane(m2_V,m1_V);
		}
		else
		{
		    // SHOW centered and moved manually

            // center
            m1_V = m1_V.rowwise() - (m1_V.colwise().sum() / m1_V.rows());
            m2_V = m2_V.rowwise() - (m2_V.colwise().sum() / m2_V.rows());
            // top2 :       40, 190, 165, 0.0125, 0.0 , 0.04
            // ear_back :   45, 160, 0, 0, 0, -0.03
            // chin : 		50, -10, -10, 0, -0.03, 0

            roughlyAlign(m2_V,xAngle,yAngle,zAngle,xT,yT,zT);

            
			//viewer.core.align_camera_center(nn_vex);
			viewer.data().show_overlay_depth = 1;
			viewer.data().show_overlay = 1;
		}

        //get_axis(my_axis);
        //viewer.data().add_points(my_axis, Eigen::RowVector3d(1, 1, 1));

		//======================================================================
		viewer.data().add_points(m1_V,Eigen::RowVector3d(0, 0, 0));
		viewer.data().add_points(m2_V,Eigen::RowVector3d(0.5,0.5,0.5));

		viewer.core.align_camera_center(m1_V, m1_F);

		viewer.data().set_face_based(true);

		viewer.data().line_width = line_width;
		viewer.data().point_size = point_size;

	}

private:

};

MyContext g_myctx;


bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{

	std::cout << "Key: " << key << " " << (unsigned int)key << std::endl;
	if (key=='q' || key=='Q')
	{
		exit(0);
	}
	return false;
}


int main(int argc, char *argv[])
{
	// Init the viewer
	igl::opengl::glfw::Viewer viewer;

	// Attach a menu plugin
	igl::opengl::glfw::imgui::ImGuiMenu menu;
	viewer.plugins.push_back(&menu);

	// menu variable Shared between two menus
	double doubleVariable = 0.1f; 

	// Add content to the default menu window via defining a Lambda expression with captures by reference([&])
	menu.callback_draw_viewer_menu = [&]()
	{
		// Draw parent menu content
		menu.draw_viewer_menu();

		// Add new group
		if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
		{
			// Expose variable directly ...
			ImGui::InputDouble("double", &doubleVariable, 0, 0, "%.4f");

			// ... or using a custom callback
			static bool boolVariable = true;
			if (ImGui::Checkbox("bool", &boolVariable))
			{
				// do something
				std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
			}

			// Expose an enumeration type
			enum Orientation { Up = 0, Down, Left, Right };
			static Orientation dir = Up;
			ImGui::Combo("Direction", (int *)(&dir), "Up\0Down\0Left\0Right\0\0");

			// We can also use a std::vector<std::string> defined dynamically
			static int num_choices = 3;
			static std::vector<std::string> choices;
			static int idx_choice = 0;
			if (ImGui::InputInt("Num letters", &num_choices))
			{
				num_choices = std::max(1, std::min(26, num_choices));
			}
			if (num_choices != (int)choices.size())
			{
				choices.resize(num_choices);
				for (int i = 0; i < num_choices; ++i)
					choices[i] = std::string(1, 'A' + i);
				if (idx_choice >= num_choices)
					idx_choice = num_choices - 1;
			}
			ImGui::Combo("Letter", &idx_choice, choices);

			// Add a button
			if (ImGui::Button("Print Hello", ImVec2(-1, 0)))
			{
				std::cout << "Hello\n";
			}
		}
	};

	// Add additional windows via defining a Lambda expression with captures by reference([&])
	menu.callback_draw_custom_window = [&]()
	{
		// Define next window position + size
		ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
		ImGui::SetNextWindowSize(ImVec2(300, 350), ImGuiSetCond_FirstUseEver);
		ImGui::Begin( "MyProperties", nullptr, ImGuiWindowFlags_NoSavedSettings );
		
		// point size
		// [event handle] if value changed
		if (ImGui::InputFloat("point_size", &g_myctx.point_size))
		{
			std::cout << "point_size changed\n";
			viewer.data().point_size = g_myctx.point_size;
		}

		// line width
		// [event handle] if value changed
		if(ImGui::InputFloat("line_width", &g_myctx.line_width))
		{
			std::cout << "line_width changed\n";
			viewer.data().line_width = g_myctx.line_width;
		}

		ImGui::SliderFloat("GaussianNoise", &g_myctx.GaussianNoise, 0, 1, "%.2f");
		ImGui::SliderFloat("SubsampleRate", &g_myctx.subsampleRate, 0, 1, "%.2f");

		//mode
		ImGui::SliderInt("Mode", &g_myctx.mode, 0,6);
        //
        ImGui::SliderFloat("x angle", &g_myctx.xAngle, -180,180);
        ImGui::SliderFloat("y angle", &g_myctx.yAngle, -180,180);
        ImGui::SliderFloat("z angle", &g_myctx.zAngle, -180,180);
		ImGui::SliderFloat("x translation", &g_myctx.xT, -1,1);
		ImGui::SliderFloat("y translation", &g_myctx.yT, -1,1);
		ImGui::SliderFloat("z translation", &g_myctx.zT, -1,1);

		//mode-text
		if (g_myctx.mode == 1) { 
			ImGui::Text("mode: ICP point to point");
		}
		else if (g_myctx.mode == 2) {
			ImGui::Text("mode: sampling ");
		}
		else if (g_myctx.mode == 3){
			ImGui::Text("mode: add Gaussian noise");
		}
        else if (g_myctx.mode == 4){
            ImGui::Text("mode: reset");
        }
        else if (g_myctx.mode == 5){
            ImGui::Text("mode: write file");
        }
		else if (g_myctx.mode == 6){
			ImGui::Text("mode: ICP point to plane");
		}
        else {
            ImGui::Text("mode: show centered and moved");
        }

        if (ImGui::Button("Apply")) {
        	std::cout << "Applying" << std::endl;
            g_myctx.reset_display(viewer);
        }
        

		ImGui::End();
	};


	// registered a event handler
	viewer.callback_key_down = &key_down;

	g_myctx.reset_display(viewer);

	// Call GUI
	viewer.launch();

}
