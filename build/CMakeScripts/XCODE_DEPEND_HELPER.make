# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.ICPAlgorithm_bin.Debug:
PostBuild.igl.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw_imgui.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.igl_opengl.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.igl.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.igl_common.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.imgui.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.glfw.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
PostBuild.glad.Debug: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin:\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Debug/libimgui.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Debug/libglfw3.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Debug/libglad.a
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Debug/ICPAlgorithm_bin


PostBuild.glad.Debug:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Debug/libglad.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Debug/libglad.a


PostBuild.glfw.Debug:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Debug/libglfw3.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Debug/libglfw3.a


PostBuild.imgui.Debug:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Debug/libimgui.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Debug/libimgui.a


PostBuild.ICPAlgorithm_bin.Release:
PostBuild.igl.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw_imgui.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.igl_opengl.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.igl.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.igl_common.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.imgui.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.glfw.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
PostBuild.glad.Release: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin:\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Release/libimgui.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Release/libglfw3.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Release/libglad.a
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/Release/ICPAlgorithm_bin


PostBuild.glad.Release:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Release/libglad.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Release/libglad.a


PostBuild.glfw.Release:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Release/libglfw3.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Release/libglfw3.a


PostBuild.imgui.Release:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Release/libimgui.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Release/libimgui.a


PostBuild.ICPAlgorithm_bin.MinSizeRel:
PostBuild.igl.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw_imgui.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.igl_opengl.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.igl.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.igl_common.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.imgui.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.glfw.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
PostBuild.glad.MinSizeRel: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin:\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/MinSizeRel/libimgui.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/MinSizeRel/libglfw3.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/MinSizeRel/libglad.a
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/MinSizeRel/ICPAlgorithm_bin


PostBuild.glad.MinSizeRel:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/MinSizeRel/libglad.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/MinSizeRel/libglad.a


PostBuild.glfw.MinSizeRel:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/MinSizeRel/libglfw3.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/MinSizeRel/libglfw3.a


PostBuild.imgui.MinSizeRel:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/MinSizeRel/libimgui.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/MinSizeRel/libimgui.a


PostBuild.ICPAlgorithm_bin.RelWithDebInfo:
PostBuild.igl.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw_imgui.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.igl_opengl_glfw.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.igl_opengl.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.igl.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.igl_common.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.imgui.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.glfw.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
PostBuild.glad.RelWithDebInfo: /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin:\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/RelWithDebInfo/libimgui.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/RelWithDebInfo/libglfw3.a\
	/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/RelWithDebInfo/libglad.a
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/RelWithDebInfo/ICPAlgorithm_bin


PostBuild.glad.RelWithDebInfo:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/RelWithDebInfo/libglad.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/RelWithDebInfo/libglad.a


PostBuild.glfw.RelWithDebInfo:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/RelWithDebInfo/libglfw3.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/RelWithDebInfo/libglfw3.a


PostBuild.imgui.RelWithDebInfo:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/RelWithDebInfo/libimgui.a:
	/bin/rm -f /Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/RelWithDebInfo/libimgui.a




# For each target create a dummy ruleso the target does not have to exist
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Debug/libglad.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/MinSizeRel/libglad.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/RelWithDebInfo/libglad.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glad/Release/libglad.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Debug/libglfw3.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/MinSizeRel/libglfw3.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/RelWithDebInfo/libglfw3.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/glfw/src/Release/libglfw3.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Debug/libimgui.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/MinSizeRel/libimgui.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/RelWithDebInfo/libimgui.a:
/Users/silviojin/compM080-compGV18-2019/courseworks/ICP/build/imgui/Release/libimgui.a:
