/*
	Copyright 2020 Limeoats
	Original project: https://github.com/Limeoats/L2DFileDialog

	Changes by Vladimir Sigalkin
*/

#pragma once

#include <filesystem>
#include <imgui.h>
#include <chrono>
#include <string>
#include <sstream>



typedef int ImGuiFileDialogType;	// -> enum ImGuiFileDialogType_        // Enum: A file dialog type

enum ImGuiFileDialogType_
{
	ImGuiFileDialogType_OpenFile,
	ImGuiFileDialogType_SaveFile,
	ImGuiFileDialogType_COUNT
};

struct ImFileDialogInfo
{
	std::string title;
	ImGuiFileDialogType type;

	std::filesystem::path fileName;
	std::filesystem::path directoryPath;
	std::filesystem::path resultPath;

	bool refreshInfo;
	size_t currentIndex;
	std::vector<std::filesystem::directory_entry> currentFiles;
	std::vector<std::filesystem::directory_entry> currentDirectories;
};

IMGUI_API bool FileDialog(bool* open, ImFileDialogInfo* dialogInfo);

<<<<<<< HEAD
#include "imgui_filedialog.hpp"
=======
#include "imgui_filedialog.cpp"
>>>>>>> 22e304b292a89f196cebb4b12e8b08d34e913988
