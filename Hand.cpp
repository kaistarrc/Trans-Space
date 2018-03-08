/**
*	Alvise Memo, 20/10/2015
*	Multimedia Technology and Telecommunications Laboratory, University of Padova
*	HandGenerator by Alvise Memo is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
*	Based on a work at http://lttm.dei.unipd.it/downloads/handposegenerator/index.html.
*	The 3D Hand Model derives from The LibHand 3D Hand Model by Marin Saric and is licensed under a Creative Commons Attribution 3.0.
*
*	\file Hand.cpp Implementation of the Hand object.
**/

#include "Hand.h"

#include <string>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <ctime>
#include <thread>
#include <chrono>
#include <iostream>
#include <cassert>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

/** Specify how much bone info to accumulate per vertex
*	IT STORES 4 COUPLE (WEIGHT-ID) FOR EACH BONES_INFO GIVEN
*	ATTENTION: ALL THE WEIGHTS GREATER THAN 0s MUST BE SENDED TO THE SHADER
*/
#define NUM_BONES_INFO 3


GLuint ogl_id_texture; // Id for the texture object
GLuint ogl_id_position_buffer, ogl_id_texture_buffer, ogl_id_bone_buffer, ogl_id_index_buffer; // Id for buffers containing vertex data (pos-tex-bone_weight) and the buffer for the indexed rendering
GLuint ogl_id_general_vao; // Id of the VertexArrayObject containing all the data

int mesh_indices_count; // Number of indices on the mesh
int mesh_base_vertex; // Number of vertices on the mesh
int mesh_base_index; // Sum of indeces count and faces count

int bone_count; // Number of different bones
int bone_to_render; // Number of bones to be rendered
std::vector<BoneInfo> bone_data; // Vector of all the bone informations
std::map<std::string, int> bone_map; // maps a bone name to its index
Rotation hand_pose_rotations; // Structure with all the informations for the hand rotation

// Stores the rule in a convenient way
struct BoneRule
{
	// Store the pair bone-id->bone-position
	struct BoneCombo
	{
		int bone_id;
		int bone_pos;

		bool operator==(BoneCombo& b) const
		{
			return this->bone_id == b.bone_id && this->bone_pos == b.bone_pos;
		}
	};

	BoneCombo bone; // The current bone
	std::vector<BoneCombo> with; // All bones that must have those position in order to the current bone to be able to take his
	std::vector<BoneCombo> without; // All bones that must NOT have those position in order to the current bone to be able to take his

	// Setup
	BoneRule()
	{
		with = std::vector<BoneCombo>();
		without = std::vector<BoneCombo>();
	}
	bool operator==(BoneRule& b) const
	{
		return this->bone == b.bone;
	}
};
std::vector<BoneRule> rules; // All bones rules for the hand

Matrix4f matrix_globalInverseTransform; // Rapresents the inverse-transform-matrix for the 3D space rapresentation

const aiScene * input_scene; // Pointer to the imported scene from ASSIMP

// This functions analyze all datas from ASSIMP imported scene, creates graphics objects (texture-buffers) and loads properties-rules-rotations
bool LoadData(std::string model_file, char* property_file, char* texture_path, bool has_rules, char* rules_path)
{
	GLenum err = glGetError();

	// Flag for OpenGL setup
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_ALPHA_TEST);

	// Create the VAO
	glGenVertexArrays(1, &ogl_id_general_vao);
	glBindVertexArray(ogl_id_general_vao);

#pragma region Load input file and get ready for initialization
	// Struct for the bones to be stored in, sorted and finally buffered to graphics memory
	struct VertexBoneData
	{
		// Pair of 4 bone-id->bone-weight
		struct SubBoneData
		{
			int IDs[4];
			float Weights[4];
		} data[NUM_BONES_INFO];

		// Single pair
		struct SubBoneTempData
		{
			int id;
			float weight;
			SubBoneTempData(int i, float v)
			{
				id = i;
				weight = v;
			}
		};

		// Temporary vector to store pairs
		std::vector<SubBoneTempData> temp_vec;

		// Initialization
		VertexBoneData()
		{
			for (int i = 0; i < NUM_BONES_INFO; i++)
			{
				memset(data[i].IDs, 0, sizeof(data[i].IDs));
				memset(data[i].Weights, 0, sizeof(data[i].Weights));
			}
		};
		void AddBoneData(int BoneID, float Weight)
		{
			temp_vec.push_back(SubBoneTempData(BoneID, Weight));
		}
		bool Comparison(SubBoneTempData i, SubBoneTempData j)
		{
			return (i.weight < j.weight);
		}
		void FinishData()
		{
			std::sort(temp_vec.begin(), temp_vec.end(), [](const SubBoneTempData & a, const SubBoneTempData & b) -> bool { return a.weight > b.weight; });

			int data_i = 0;
			float w = 0.0f;
			for (int i = 0; i < NUM_BONES_INFO; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					if (data_i >= temp_vec.size())
					{
						assert(fabs(w - 1.0f) < 0.01f);
						return;
					}

					data[i].IDs[j] = temp_vec[data_i].id;
					data[i].Weights[j] = temp_vec[data_i].weight;
					w += data[i].Weights[j];

					data_i++;
				}
			}

			int a = 0;
		}
	};

	Assimp::Importer imp;
	input_scene = imp.ReadFile(model_file, aiProcess_Triangulate);
	assert(input_scene);
	input_scene = imp.GetOrphanedScene();

	matrix_globalInverseTransform = input_scene->mRootNode->mTransformation;
	matrix_globalInverseTransform.Inverse();

	std::vector<float> Positions;
	std::vector<float> TexCoords;
	std::vector<VertexBoneData> Bones;
	std::vector<int> Indices;

	int NumVertices = 0;
	int NumIndices = 0;

	// Count the number of vertices and indices
	mesh_indices_count = input_scene->mMeshes[0]->mNumFaces * 3;
	mesh_base_vertex = NumVertices;
	mesh_base_index = NumIndices;

	NumVertices += input_scene->mMeshes[0]->mNumVertices;
	NumIndices += mesh_indices_count;


	// Reserve space in the vectors for the vertex attributes and indices
	Positions.reserve(NumVertices * 3);
	TexCoords.reserve(NumVertices * 2);
	Bones.resize(NumVertices);
	Indices.reserve(NumIndices);
	//printf("numvertices numindices %d %d\n", NumVertices, NumIndices);
#pragma endregion

	//std::cout << "LoadData1" << std::endl;

#pragma region Populate the vertex attribute vectors
	// Initialize the meshes in the scene one by one
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);


	for (unsigned int i = 0; i < input_scene->mMeshes[0]->mNumVertices; i++)
	{
		const aiVector3D* pPos = &(input_scene->mMeshes[0]->mVertices[i]);
		const aiVector3D* pNormal = &(input_scene->mMeshes[0]->mNormals[i]);
		//const aiVector3D* pTexCoord = input_scene->mMeshes[0]->HasTextureCoords(0) ? &(input_scene->mMeshes[0]->mTextureCoords[0][i]) : &Zero3D;
		const aiVector3D* pTexCoord = &(input_scene->mMeshes[0]->mTextureCoords[0][i]);


		Positions.push_back(pPos->x);
		Positions.push_back(pPos->y);
		Positions.push_back(pPos->z);

		TexCoords.push_back(pTexCoord->x);
		TexCoords.push_back(pTexCoord->y);
		//printf("vertex x:%f y:%f z:%f\n", pPos->x, pPos->y, pPos->z);
		//printf("texture x:%f y:%f\n", pTexCoord->x, pTexCoord->y);
	}
#pragma endregion

	//std::cout << "LoadData2" << std::endl;
	//printf("size:%d\n", input_scene->mMeshes[0]->mNumVertices);
#pragma region LoadBones(MeshIndex, input_scene->mMeshes[0], Bones);
	//printf("initialize bone\n");
	//printf("mnumbones:%d\n", input_scene->mMeshes[0]->mNumBones);
	for (unsigned int i = 0; i < input_scene->mMeshes[0]->mNumBones; i++)
	{
		int BoneIndex = 0;
		std::string BoneName(input_scene->mMeshes[0]->mBones[i]->mName.data);
		//printf("[%d]bonename:%s\n", i,BoneName);
		if (bone_map.find(BoneName) == bone_map.end())
		{
			// Allocate an index for a new bone
			BoneIndex = bone_count;
			bone_count++;
			BoneInfo bi;
			bone_data.push_back(bi);
			bone_data[BoneIndex].BoneOffset = input_scene->mMeshes[0]->mBones[i]->mOffsetMatrix;

			bone_data[BoneIndex].bonePosition.x = bone_data[BoneIndex].BoneOffset.m[0][3];
			bone_data[BoneIndex].bonePosition.y = bone_data[BoneIndex].BoneOffset.m[1][3];
			bone_data[BoneIndex].bonePosition.z = bone_data[BoneIndex].BoneOffset.m[2][3];
			bone_data[BoneIndex].bonePosition.w = 1;

			bone_data[BoneIndex].name = BoneName;
			bone_map[BoneName] = BoneIndex;
		}
		else {
			BoneIndex = bone_map[BoneName];
		}

		for (unsigned int j = 0; j < input_scene->mMeshes[0]->mBones[i]->mNumWeights; j++) {
			int VertexID = mesh_base_vertex + input_scene->mMeshes[0]->mBones[i]->mWeights[j].mVertexId;
			float Weight = input_scene->mMeshes[0]->mBones[i]->mWeights[j].mWeight;
			Bones[VertexID].AddBoneData(BoneIndex, Weight);

			//if(Weight>0.0f)
			if(Weight >0.7f)
			{
				//printf("boneindex:%d more than weight\n", BoneIndex);
				bone_data[BoneIndex].pos.x += input_scene->mMeshes[0]->mVertices[VertexID].x;// * Weight;
				bone_data[BoneIndex].pos.y += input_scene->mMeshes[0]->mVertices[VertexID].y;// * Weight;
				bone_data[BoneIndex].pos.z += input_scene->mMeshes[0]->mVertices[VertexID].z;// * Weight;
				bone_data[BoneIndex].pos_count++;
			}

		}
	}
	std::vector<int> ff;
	for (int i = 0; i < Bones.size(); i++)
	{
		ff.push_back((int)Bones[i].temp_vec.size());

		std::vector<float> l;

		bool r = false;

		Bones[i].data[0].Weights[1] = 1.0f;
		for (int j = 1; j < 4; j++)
			Bones[i].data[0].Weights[j] = 0.0f;

		if (r)
		{
			l.push_back(1.0f);
		}


	}
	std::sort(ff.begin(), ff.end());
	for (int i = 0; i < Bones.size(); i++)
		Bones[i].FinishData();
	//printf("bonesize:%d\n", Bones.size());
	//printf("bone_datasize:%d\n", bone_data.size());
	for (int i = 0; i < bone_data.size(); i++)
	{
		//printf("i:%d\n", i);
		bone_data[i].pos.x /= bone_data[i].pos_count;
		bone_data[i].pos.y /= bone_data[i].pos_count;
		bone_data[i].pos.z /= bone_data[i].pos_count;
		bone_data[i].pos.w = 1.0f;
		//printf("bone pos[%d]: %f %f %f\n", i, bone_data[i].pos.x, bone_data[i].pos.y, bone_data[i].pos.z);
	}
#pragma endregion

	//std::cout << "LoadData3" << std::endl;

#pragma region Populate the index buffer
	for (unsigned int i = 0; i < input_scene->mMeshes[0]->mNumFaces; i++) {
		const aiFace& Face = input_scene->mMeshes[0]->mFaces[i];
		assert(Face.mNumIndices == 3);
		Indices.push_back(Face.mIndices[0]);
		Indices.push_back(Face.mIndices[1]);
		Indices.push_back(Face.mIndices[2]);
	}
#pragma endregion

	//std::cout << "LoadData4" << std::endl;

#pragma region InitMaterials(input_scene, model_file) Extract the directory part from the file name
	// Initialize the materials
	aiString Path;
	assert(input_scene->mNumMaterials > 1);
	assert(input_scene->mMaterials[1]->GetTextureCount(aiTextureType_DIFFUSE) > 0);
	assert(input_scene->mMaterials[1]->GetTexture(aiTextureType_DIFFUSE, 0, &Path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS);

	// Data read from the header of the BMP file
	unsigned char header[54]; // Each BMP file begins by a 54-bytes header
	int dataPos;     // Position in the file where the actual data begins
	int width, height;
	int imageSize;
	unsigned char * data;
	//FILE * file;
	//fopen_s(&file, texture_path,"rb");
	FILE* file = fopen(texture_path, "rb");
	assert(file); // CANNOT OPEN FILE
	data = new unsigned char[2048 * 2048 * 3];
	fread(data, 1, 2048 * 2048 * 3, file);
	fclose(file);
	/* TODO: WEIRD PROBLEM FOR RELEASE DEBUG ON WIN x64
	FILE* file = fopen(texture_path,"rb");
	assert(file); // CANNOT OPEN FILE
	assert(fread(header, 1, 54, file)==54);
	assert(header[0]=='B' && header[1]=='M');
	dataPos    = *(int*)&(header[0x0A]);
	imageSize  = *(int*)&(header[0x22]);
	width      = *(int*)&(header[0x12]);
	height     = *(int*)&(header[0x16]);
	data = new unsigned char [imageSize];
	fread(data,1,imageSize,file);
	fclose(file);
	*/

	// Create one OpenGL texture
	glGenTextures(1, &ogl_id_texture);
	//err = glGetError(); if (err != GL_NO_ERROR) { std::cout << "LoadData ERROR glGenTextures" << std::endl; return false; }
	glBindTexture(GL_TEXTURE_2D, ogl_id_texture);
	//err = glGetError(); if (err != GL_NO_ERROR) { std::cout << "LoadData ERROR glBindTexture " << std::endl; return false; }
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2048, 2048, 0, GL_BGR, GL_UNSIGNED_BYTE, data);
	//err = glGetError(); if (err != GL_NO_ERROR) { std::cout << "LoadData ERROR glTexImage2D " << width << " " << height << " " << imageSize << " " << dataPos << " " << err << std::endl; return false; }
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//err = glGetError(); if (err != GL_NO_ERROR) { std::cout << "LoadData ERROR" << std::endl; return false; }
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	//err = glGetError(); if (err != GL_NO_ERROR) { std::cout << "LoadData ERROR" << std::endl; return false; }
	glBindTexture(GL_TEXTURE_2D, 0);
	//err = glGetError(); if (err != GL_NO_ERROR) { std::cout << "LoadData ERROR" << std::endl; return false; }

	delete[] data;
#pragma endregion

	//std::cout << "LoadData5" << std::endl;

#pragma region Generate and populate the buffers with vertex attributes and the indices
	// Buffer for vertex data, X-Y-Z float
	glGenBuffers(1, &ogl_id_position_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, ogl_id_position_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)* Positions.size(), &Positions[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(0); // Vertex array position 0
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);   // 3 float, not normalized  

	glGenBuffers(1, &ogl_id_texture_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, ogl_id_texture_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)* TexCoords.size(), &TexCoords[0], GL_STATIC_DRAW);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &ogl_id_bone_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, ogl_id_bone_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Bones[0]) * Bones.size(), &Bones[0], GL_STATIC_DRAW);

	for (int i = 0; i < NUM_BONES_INFO; i++)
	{
		glEnableVertexAttribArray(2 + i * 2);
		glVertexAttribIPointer(2 + i * 2, 4, GL_INT, sizeof(VertexBoneData), (const GLvoid*)(i * 32));
		glEnableVertexAttribArray(3 + i * 2);
		glVertexAttribPointer(3 + i * 2, 4, GL_FLOAT, GL_FALSE, sizeof(VertexBoneData), (const GLvoid*)(i * 32 + 16));
	}


	glGenBuffers(1, &ogl_id_index_buffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ogl_id_index_buffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices[0]) * Indices.size(), &Indices[0], GL_STATIC_DRAW);
#pragma endregion

	//std::cout << "LoadData6" << std::endl;

#pragma region Load or generate positions file
	//std::ifstream property_in;
	//property_in.open(property_file);
	//if (property_in.is_open())
	//{
	//	std::string name;
	//	float rx, ry, rz;
	//	int line_n = 0;
	//	while (property_in >> name >> rx >> ry >> rz)
	//	{
	//		if (name == "rotationx") hand_pose_rotations.rot_x.push_back(rx);
	//		else if (name == "rotationy") hand_pose_rotations.rot_y.push_back(ry);
	//		else if (name == "rotationz") hand_pose_rotations.rot_z.push_back(rz);
	//		else
	//		{
	//			int index = bone_map[name];
	//			BoneInfo::BoneSetup bs;
	//			bs.imp_rot_x = rx;
	//			bs.imp_rot_y = ry;
	//			bs.imp_rot_z = rz;
	//			bs.impRot = Matrix4f::MakeRotationMatrix(rx, ry, rz);
	//			bone_data[index].possible_positions.push_back(bs);
	//		}
	//	}
	//}
	//else
	//{
	//printf("size:%d\n", bone_data.size());
	for (int i = 0; i < bone_data.size(); i++)
	{
		BoneInfo::BoneSetup bs;
		bs.imp_rot_x = 0.0f;
		bs.imp_rot_y = 0.0f;
		bs.imp_rot_z = 0.0f;
		bs.impRot = Matrix4f::MakeRotationMatrix(0.0f, 0.0f, 0.0f);
		bone_data[i].possible_positions.push_back(bs);
	}
	//}
	//property_in.close();

#pragma endregion

	//std::cout << "LoadData7" << std::endl;

#pragma region Load rules if presents
	//rules = std::vector<BoneRule>();
	//if (has_rules)
	//{
	//	std::ifstream rules_in;
	//	rules_in.open(rules_path);
	//	if (rules_in.is_open())
	//	{
	//		std::string name_from;
	//		int index_from;
	//		int pos_from;
	//		char flag;
	//		std::string name_to;
	//		int index_to;
	//		int pos_to;

	//		int rule_index;

	//		while (rules_in >> name_from >> pos_from >> flag >> name_to >> pos_to)
	//		{
	//			index_from = bone_map[name_from];
	//			index_to = bone_map[name_to];

	//			assert(flag == 'w' || flag == 'o');
	//			assert(index_from != -1 && index_to != -1);
	//			assert(bone_data[index_from].possible_positions.size() > pos_from);
	//			assert(bone_data[index_to].possible_positions.size() > pos_to);

	//			rule_index = -1;
	//			for(int i = 0; i < rules.size(); i++)
	//			{
	//				if (rules[i].bone.bone_id == index_from && rules[i].bone.bone_pos == pos_from)
	//				{
	//					rule_index = i;
	//					break;
	//				}
	//			}

	//			if (rule_index == -1)
	//			{
	//				BoneRule b;
	//				b.bone.bone_id = index_from;
	//				b.bone.bone_pos = pos_from;
	//				rules.push_back(b);
	//				rule_index = (int)rules.size() - 1;
	//			}

	//			BoneRule::BoneCombo to_add;
	//			to_add.bone_id = index_to;
	//			to_add.bone_pos = pos_to;

	//			if (flag == 'w')
	//				rules[rule_index].with.push_back(to_add);
	//			else
	//				rules[rule_index].without.push_back(to_add);
	//		}
	//	}
	//	rules_in.close();
	//}
#pragma endregion

	//std::cout << "LoadData8" << std::endl;

	err = glGetError(); if (err != GL_NO_ERROR) { std::cout << "LoadData ERROR" << std::endl; return false; }

	// Make sure the VAO is not changed from the outside
	glBindVertexArray(0);

	return true;
}
void ReadNodeHeirarchy(float AnimationTime, const aiNode* pNode, const Matrix4f& ParentTransform)
{
	std::string NodeName(pNode->mName.data);
	const aiAnimation* pAnimation = input_scene->mAnimations[0];
	Matrix4f NodeTransformation(pNode->mTransformation);

	const aiNodeAnim* node_ptr = NULL;
	{
		for (unsigned int i = 0; i < pAnimation->mNumChannels; i++) {
			const aiNodeAnim* pNodeAnim_t = pAnimation->mChannels[i];

			if (std::string(pNodeAnim_t->mNodeName.data) == NodeName) {
				node_ptr = pAnimation->mChannels[i];
				break;
			}
		}
	}

#pragma region COMPUTE SINGLE NODE TRANSFORMATIONS
	if (node_ptr)
	{
		// Interpolate rotation and generate rotation transformation matrix
		aiQuaternion RotationQ;
		{
			// we need at least two values to interpolate...
			if (node_ptr->mNumRotationKeys == 1)
			{
				RotationQ = node_ptr->mRotationKeys[0].mValue;
			}
			else
			{
				int RotationIndex = -1;
				//FindRotation(AnimationTime, node_ptr);
				{
					assert(node_ptr->mNumRotationKeys > 0);

					for (unsigned int i = 0; i < node_ptr->mNumRotationKeys - 1; i++) {
						if (AnimationTime < (float)node_ptr->mRotationKeys[i + 1].mTime) {
							RotationIndex = i;
							break;
						}
					}
					assert(RotationIndex != -1);
				}
				int NextRotationIndex = (RotationIndex + 1);
				//assert(NextRotationIndex < node_ptr->mNumRotationKeys);
				float DeltaTime = (float)(node_ptr->mRotationKeys[NextRotationIndex].mTime - node_ptr->mRotationKeys[RotationIndex].mTime);
				float Factor = (AnimationTime - (float)node_ptr->mRotationKeys[RotationIndex].mTime) / DeltaTime;
				//assert(Factor >= 0.0f && Factor <= 1.0f);
				const aiQuaternion& StartRotationQ = node_ptr->mRotationKeys[RotationIndex].mValue;
				const aiQuaternion& EndRotationQ = node_ptr->mRotationKeys[NextRotationIndex].mValue;
				aiQuaternion::Interpolate(RotationQ, StartRotationQ, EndRotationQ, Factor);
				RotationQ = RotationQ.Normalize();
			}
		}
		Matrix4f RotationM = Matrix4f(RotationQ.GetMatrix());

		// Interpolate translation and generate translation transformation matrix
		aiVector3D Translation;
		{
			if (node_ptr->mNumPositionKeys == 1)
			{
				Translation = node_ptr->mPositionKeys[0].mValue;
			}
			else
			{
				int PositionIndex = -1;
				//FindPosition(AnimationTime, node_ptr);
				{
					for (unsigned int i = 0; i < node_ptr->mNumPositionKeys - 1; i++) {
						if (AnimationTime < (float)node_ptr->mPositionKeys[i + 1].mTime) {
							PositionIndex = i;
							break;
						}
					}
					assert(PositionIndex != -1);
				}
				int NextPositionIndex = (PositionIndex + 1);
				//assert(NextPositionIndex < node_ptr->mNumPositionKeys);
				float DeltaTime = (float)(node_ptr->mPositionKeys[NextPositionIndex].mTime - node_ptr->mPositionKeys[PositionIndex].mTime);
				float Factor = (AnimationTime - (float)node_ptr->mPositionKeys[PositionIndex].mTime) / DeltaTime;
				//assert(Factor >= 0.0f && Factor <= 1.0f);
				const aiVector3D& Start = node_ptr->mPositionKeys[PositionIndex].mValue;
				const aiVector3D& End = node_ptr->mPositionKeys[NextPositionIndex].mValue;
				aiVector3D Delta = End - Start;
				Translation = Start + Factor * Delta;
			}
		}
		NodeTransformation = Matrix4f::MakeTranslationMatrix(Translation.x, Translation.y, Translation.z) * RotationM;
	}
#pragma endregion

	Matrix4f GlobalTransformation = ParentTransform * NodeTransformation;

	if (bone_map.find(NodeName) != bone_map.end()) {
		int BoneIndex = bone_map[NodeName];
		bone_data[BoneIndex].FinalTransformation = matrix_globalInverseTransform * GlobalTransformation * bone_data[BoneIndex].BoneOffset;
	}

	for (unsigned int i = 0; i < pNode->mNumChildren; i++) {
		ReadNodeHeirarchy(AnimationTime, pNode->mChildren[i], GlobalTransformation);
	}
}
void ComputeBoneAnimation(float TimeInSeconds, std::vector<Matrix4f>& Transforms)
{
	Matrix4f Identity;
	Identity.IdentityMatrix();

	float TicksPerSecond = (float)(input_scene->mAnimations[0]->mTicksPerSecond != 0 ? input_scene->mAnimations[0]->mTicksPerSecond : 25.0f);
	float TimeInTicks = TimeInSeconds * TicksPerSecond;
	float AnimationTime = fmod(TimeInTicks, (float)input_scene->mAnimations[0]->mDuration);

	ReadNodeHeirarchy(AnimationTime, input_scene->mRootNode, Identity); //time consuming

	Transforms.resize(bone_count);

	for (int i = 0; i < bone_count; i++) {
		Transforms[i] = bone_data[i].FinalTransformation;
	}
}
void ElaborateBonesPositions(const aiNode* pNode, const Matrix4f& ParentTransform)
{
	//printf("--elaborate function--\n");
	std::string NodeName(pNode->mName.data);

	Matrix4f NodeTransformation(pNode->mTransformation);


	Matrix4f GlobalTransformation = ParentTransform * NodeTransformation;

	if (bone_map.find(NodeName) != bone_map.end())
	{
		int BoneIndex = bone_map[NodeName];
		
		GlobalTransformation = GlobalTransformation * bone_data[BoneIndex].possible_positions[bone_data[BoneIndex].current_position].impRot;
		bone_data[BoneIndex].FinalTransformation = matrix_globalInverseTransform * GlobalTransformation * bone_data[BoneIndex].BoneOffset;
	
		bone_data[BoneIndex].tempMat = GlobalTransformation;
	}

	for (unsigned int i = 0; i < pNode->mNumChildren; i++)
		ElaborateBonesPositions(pNode->mChildren[i], GlobalTransformation);


	/*
	std::string NodeName(pNode->mName.data);

	Matrix4f NodeTransformation(pNode->mTransformation);


	Matrix4f GlobalTransformation = ParentTransform * NodeTransformation;

	if (bone_map.find(NodeName) != bone_map.end())
	{
	int BoneIndex = bone_map[NodeName];
	GlobalTransformation = GlobalTransformation * bone_data[BoneIndex].possible_positions[bone_data[BoneIndex].current_position].impRot;
	bone_data[BoneIndex].FinalTransformation = matrix_globalInverseTransform * GlobalTransformation * bone_data[BoneIndex].BoneOffset;

	}

	for (unsigned int i = 0 ; i < pNode->mNumChildren ; i++)
	ElaborateBonesPositions(pNode->mChildren[i], GlobalTransformation);
	*/
}
void ComputeBoneStatic(std::vector<Matrix4f>& Transforms)
{
	Matrix4f Identity;
	Identity.IdentityMatrix();

	ElaborateBonesPositions(input_scene->mRootNode, Identity);
	//ElaborateBonesPositions(input_scene->mRootNode->mChildren[0]->mChildren[0], Identity);
	//ElaborateBonesPositions(input_scene->mRootNode->mChildren[0]->mChildren[0], Identity);

	Transforms.resize(bone_count);

	//distance
	/*
	for (int i = 0; i < bone_count-1; i++){
		float x0 = bone_data[i].pos.x;
		float y0 = bone_data[i].pos.y;
		float z0 = bone_data[i].pos.z;
		float x1 = bone_data[i+1].pos.x;
		float y1 = bone_data[i+1].pos.y;
		float z1 = bone_data[i+1].pos.z;

		float dx = x0 - x1; float dy = y0 - y1; float dz = z0 - z1;
		float dist = sqrt(dx*dx + dy*dy + dz*dz);
		printf("0dist[%d]=%f\n", i, dist);
	}
	*/

	for (int i = 0; i < bone_count; i++)
	{
		//printf("check[%d]\n", i);
		//printf("pos[%d]=%.2f %.2f %.2f\n", i,bone_data[i].pos.x, bone_data[i].pos.y, bone_data[i].pos.z);
		Transforms[i] = bone_data[i].FinalTransformation;
		//printf("Transforms:%f\n", Transforms[i].m[0][0]);

		bone_data[i].pos_transformed = Transforms[i] * bone_data[i].pos;
		bone_data[i].bonePosition_transformed = Transforms[i] * bone_data[i].bonePosition;
		//printf("bone_data[i].pos:%f\n",bone_data[i].pos.x);
		//printf("[%d]%f %f %f\n", i, bone_data[i].pos_transformed.x, bone_data[i].pos_transformed.y, bone_data[i].pos_transformed.z);
		//printf("\n");
	}

	

	//printf("compute bone static\n");
}
void RenderBones()
{
	for (int i = 0; i < 25; i++)
	{
		glTranslatef(bone_data[i].pos_transformed.x, bone_data[i].pos_transformed.y, bone_data[i].pos_transformed.z);
		glColor3f(1.0f, 0.0f, 1.0f);
		//glutWireSphere(2, 2, 2);
		glutSolidSphere(2, 2, 2);
	}
	
}

Hand::Hand(void)
{
	//matrix_model = Matrix4f::MakeRotationMatrix(0.0f, 0.0f, -90.0f) * Matrix4f::MakeTranslationMatrix(0.0f, 0.0f, 0.0f);
	
	
	joints[0] = "R_thumb_meta";
	joints[1] = "R_thumb_a";
	joints[2] = "R_thumb_b";

	joints[3] = "R_index_a";
	joints[4] = "R_index_b";
	joints[5] = "R_index_c";

	joints[6] = "R_middle_a";
	joints[7] = "R_middle_b";
	joints[8] = "R_middle_c";

	joints[9] = "R_ring_a";
	joints[10] = "R_ring_b";
	joints[11] = "R_ring_c";

	joints[12] = "R_pinky_a";
	joints[13] = "R_pinky_b";
	joints[14] = "R_pinky_c";
	
	/*
	joints[0] = "finger5joint1";
	joints[1] = "finger5joint2";
	joints[2] = "finger5joint3";
	joints[3] = "Bone.003";
	joints[4] = "finger4joint1";
	joints[5] = "finger4joint2";
	joints[6] = "finger4joint3";
	joints[7] = "Bone.002";
	joints[8] = "finger3joint1";
	joints[9] = "finger3joint2";
	joints[10] = "finger3joint3";
	joints[11] = "Bone.001";
	joints[12] = "finger2joint1";
	joints[13] = "finger2joint2";
	joints[14] = "finger2joint3";
	joints[15] = "Bone";
	joints[16] = "finger1joint1";
	joints[17] = "finger1joint2";
	joints[18] = "finger1joint3";
	*/
}
Hand::~Hand(void)
{
}

bool Hand::Init(HandParameters parameters)
{
	//std::cout << "Hand::Init1" << std::endl;

	// Reading input parameters
	//param = parameters;
	parameters.CopyTo(&param);

	//projection matrix
	Matrix4f::PersProjInfo persProjInfo;
	persProjInfo.Height = param.height;
	persProjInfo.Width = param.width;
	persProjInfo.zNear = param.render_near;
	persProjInfo.zFar = param.render_far;
	persProjInfo.fx = param.fx;
	persProjInfo.fy = param.fy;
	persProjInfo.cx = param.cx;
	persProjInfo.cy = param.cy;
	persProjInfo.FOV = param.render_FOV;

	//printf("fov:param.render_fov:%f\n", param.render_FOV);
	//matrix_projection = Matrix4f::MakeProjectionMatrix(persProjInfo);
	matrix_projection = Matrix4f::Set_GL_PROJECTION(persProjInfo);
	//for (int i = 0; i < 4;i++)
	//for (int j = 0; j < 4; j++)
	//	printf("[%d][%d]:%.2f\n", j, i, matrix_projection.m[j][i]);

	// Enabling some OpenGL features needed
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_ALPHA_TEST);

	// Simple structure to load, compile and link shaders, thanks to http://ogldev.atspace.co.uk/
	struct helper
	{
		std::vector<GLuint> shader_obj;
		bool AddShader(GLenum m_shaderProg, GLenum ShaderType, const char* pFilename)
		{
			std::string s;
			std::ifstream f(pFilename);
			assert(f.is_open());
			std::string line;
			while (std::getline(f, line))
			{
				s.append(line);
				s.append("\n");
			}
			f.close();

			GLuint ShaderObj = glCreateShader(ShaderType);

			if (ShaderObj == 0) {
				fprintf(stderr, "Error creating shader type %d\n", ShaderType);
				return false;
			}

			// Save the shader object - will be deleted in the destructor
			shader_obj.push_back(ShaderObj);

			const GLchar* p[1];
			p[0] = s.c_str();
			GLint Lengths[1] = { (GLint)s.size() };

			glShaderSource(ShaderObj, 1, p, Lengths);

			glCompileShader(ShaderObj);

			GLint success;
			glGetShaderiv(ShaderObj, GL_COMPILE_STATUS, &success);

			if (!success) {
				GLchar InfoLog[1024];
				glGetShaderInfoLog(ShaderObj, 1024, NULL, InfoLog);
				fprintf(stderr, "Error compiling '%s': '%s'\n", pFilename, InfoLog);
				return false;
			}

			glAttachShader(m_shaderProg, ShaderObj);

			return true;
		}
		bool Finalize(GLenum m_shaderProg)
		{
			GLint Success = 0;
			GLchar ErrorLog[1024] = { 0 };

			glLinkProgram(m_shaderProg);

			glGetProgramiv(m_shaderProg, GL_LINK_STATUS, &Success);
			if (Success == 0) {
				glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
				fprintf(stderr, "Error linking shader program: '%s'\n", ErrorLog);
				return false;
			}

			glValidateProgram(m_shaderProg);
			glGetProgramiv(m_shaderProg, GL_VALIDATE_STATUS, &Success);
			if (!Success) {
				glGetProgramInfoLog(m_shaderProg, sizeof(ErrorLog), NULL, ErrorLog);
				fprintf(stderr, "Invalid shader program: '%s'\n", ErrorLog);
				//   return false;
			}

			// Delete the intermediate shader objects that have been added to the program
			for (int i = 0; i < shader_obj.size(); i++)
			{
				glDeleteShader(shader_obj[i]);
			}

			shader_obj.clear();

			return (glGetError() == GL_NO_ERROR);
		}
	} help;
	shader_id = glCreateProgram();
	if (!help.AddShader(shader_id, GL_VERTEX_SHADER, param.setup_vertexShaderPath))
	{
		std::cout << "Vertex shader ERROR. Aborting initiation." << std::endl;
		return false;
	}
	if (!help.AddShader(shader_id, GL_FRAGMENT_SHADER, param.setup_fragmentShaderPath))
	{
		std::cout << "Fragment shader ERROR. Aborting initiation." << std::endl;
		return false;
	}
	if (!help.Finalize(shader_id))
	{
		std::cout << "Shader linking ERROR. Aborting initiation." << std::endl;
		return false;
	}

	//std::cout << "Hand::Init2" << std::endl;

	// Initializing uniforms locations
	shader_modelViewProj_loc = glGetUniformLocation(shader_id, param.setup_shader_modelViewProjUniformName);
	shader_modelView_loc = glGetUniformLocation(shader_id, param.setup_shader_modelViewUniformName);
	shader_texture_loc = glGetUniformLocation(shader_id, param.setup_shader_textureUniformName);
	for (int i = 0; i < sizeof(shader_bones_loc) / sizeof(shader_bones_loc[0]); i++) {
		std::ostringstream oss;
		oss << param.setup_shader_boneUniformName << '[' << i << ']';
		shader_bones_loc[i] = glGetUniformLocation(shader_id, oss.str().c_str());
	}
	vismode_loc = glGetUniformLocation(shader_id, "vis_mode");
	glUseProgram(shader_id);
	glUniform1i(shader_texture_loc, 0);
	glUseProgram(0);

	//std::cout << "Hand::Init3" << std::endl;

	// Calling the LoadMesh function to retrieve the model from the collada export format
	if (param.render_use_full_model)
		return LoadData(param.setup_model_path_full, param.setup_model_property_path, param.setup_modelTexture_path, param.general_use_rules, param.setup_model_rules_path);
	else
		return LoadData(param.setup_model_path_low, param.setup_model_property_path, param.setup_modelTexture_path, param.general_use_rules, param.setup_model_rules_path);
}

void Hand::initialRun()
{

}

void Hand::setViewport(int w, int h){
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
}

void Hand::setViewport(int px, int py, int w, int h){
	//glViewport(w*px, h*py, (GLsizei)w, (GLsizei)h);
	glViewport(w*px, h*(param.particle_numy - py - 1), w, h);

}

void Hand::Render(float r_x, float r_y, float r_z, bool cont_rot, float wx, float wy, float wz, std::string vistype)
{


	glUseProgram(shader_id);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_ALPHA_TEST);

	runningTime += param.render_animationSpeed;

	std::vector<Matrix4f> Transforms;
	if (param.general_animation_on)
		ComputeBoneStatic(Transforms);
	else
		ComputeBoneAnimation(runningTime, Transforms);

	for (int i = 0; i < Transforms.size(); i++)
		glUniformMatrix4fv(shader_bones_loc[i], 1, GL_TRUE, (const GLfloat*)Transforms[i]);



	//matrix_model = Matrix4f::MakeRotationMatrix(r_x, r_y, r_z) * Matrix4f::MakeScalingMatrix(15, 15, 13);
	//matrix_model = Matrix4f::MakeRotationMatrix(r_x, r_y, r_z) * Matrix4f::MakeScalingMatrix(12, 11, 13);//(13,13,13)
	matrix_model = Matrix4f::MakeRotationMatrix(r_x, r_y, r_z) * Matrix4f::MakeScalingMatrix(param.sx_palm, param.sy_palm, param.sz_palm);
	matrix_translation = Matrix4f::MakeTranslationMatrix(wx, wy,wz);
	matrix_modelViewProj = matrix_projection * matrix_translation * matrix_model;
	matrix_modelView = matrix_translation*matrix_model;
	

	if (vistype.compare("color") == 0)
		glUniform1i(vismode_loc, 0);
	else
		glUniform1i(vismode_loc, 1);


	glUniformMatrix4fv(shader_modelViewProj_loc, 1, GL_FALSE, (const GLfloat*)matrix_modelViewProj);
	glUniformMatrix4fv(shader_modelView_loc, 1, GL_FALSE, (const GLfloat*)matrix_modelView);

	glBindVertexArray(ogl_id_general_vao);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ogl_id_texture);
	glDrawElementsBaseVertex(GL_TRIANGLES, mesh_indices_count, GL_UNSIGNED_INT, (void*)(sizeof(int)* mesh_base_index), mesh_base_vertex);

	glBindVertexArray(0);

	glUseProgram(0);

	if (param.render_bone_sight)
	{
		Matrix4f mvp_t = matrix_modelViewProj.Transpose();

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(mvp_t.m[0]);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		RenderBones();
	}

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_ALPHA_TEST);

}
void Hand::SetJoint(int jn, int pc)
{
	assert(bone_map.find(joints[jn]) != bone_map.end());
	int BoneIndex = bone_map[joints[jn]];
	bone_data[BoneIndex].current_position = pc;
	bone_to_render = BoneIndex;

}
void Hand::SetJoint(int jn, int pc, float x, float y, float z)
{
	//printf("--setjoint function--\n");
	assert(bone_map.find(joints[jn]) != bone_map.end());
	int BoneIndex = bone_map[joints[jn]];
	
	bone_data[BoneIndex].possible_positions[pc].impRot = Matrix4f::MakeRotationMatrix(x, y, z);

	float sx = 1.0;
	float sy = 1.0;
	float sz = 1.0;

	/*
	if (jn == 0)
		sy = 1.1;
	if (jn == 3)
		sy = 1.1;
	if (jn == 6)
		sy = 1.1;
	if (jn == 9)
		sy = 1.1;
	if (jn == 12)
		sy = 1.1;
	*/


	bone_data[BoneIndex].possible_positions[pc].impRot = Matrix4f::MakeRotationMatrix(x, y, z)*Matrix4f::MakeScalingMatrix(sx,sy,sz);//(1,1.08,1);

	bone_data[BoneIndex].possible_positions[pc].imp_rot_x = x;
	bone_data[BoneIndex].possible_positions[pc].imp_rot_y = y;
	bone_data[BoneIndex].possible_positions[pc].imp_rot_z = z;

	bone_data[BoneIndex].current_position = pc;
	bone_to_render = BoneIndex;
}

void Hand::GetJointPosition(int fi, int ji, float* out)
{
	
	
	int jn = 3 * fi + ji;
	assert(bone_map.find(joints[jn]) != bone_map.end());
	int BoneIndex = bone_map[joints[jn]];

	BoneIndex += 1;
	printf("[%d]joints name:%s, boneindex:%d\n", jn,joints[jn], BoneIndex);

	Matrix4f mvp_t = matrix_modelViewProj.Transpose();
	bone_data[BoneIndex].pos_transformed = matrix_modelView*bone_data[BoneIndex].pos_transformed;
	//bone_data[BoneIndex].pos_transformed =mvp_t*bone_data[BoneIndex].pos_transformed;
	
	out[0] = bone_data[BoneIndex].pos_transformed.x;
	out[1] = -bone_data[BoneIndex].pos_transformed.y;
	out[2] = bone_data[BoneIndex].pos_transformed.z;
}

void Hand::GetJointAllPosition(std::vector<float>* out){

	//printf("bonemapsize:%d\n", bone_map.size());
	for (int i = 0; i<bone_map.size(); i++)
	{
	
		/*
		bone_data[i].pos_transformed = matrix_modelView*bone_data[i].pos_transformed;
		bone_data[i].pos_transformed = matrix_modelView*bone_data[i].pos;
		float x = bone_data[i].pos_transformed.x;
		float y = -bone_data[i].pos_transformed.y;
		float z = bone_data[i].pos_transformed.z;
		*/

		//Matrix4f mat = matrix_modelView*bone_data[i].FinalTransformation;
		
		//Matrix4f mat = matrix_modelView*bone_data[i].FinalTransformation;
		Matrix4f mat = matrix_modelView*matrix_globalInverseTransform*bone_data[i].tempMat;
		float x = mat.m[0][3];
		float y = -mat.m[1][3];
		float z = mat.m[2][3];
		

		out->push_back(x);
		out->push_back(y);
		out->push_back(z);
		

	}

}



void Hand::GetJoint(int jn, int pc, float &x, float &y, float &z)
{
	assert(bone_map.find(joints[jn]) != bone_map.end());
	int BoneIndex = bone_map[joints[jn]];
	x = bone_data[BoneIndex].possible_positions[pc].imp_rot_x;
	y = bone_data[BoneIndex].possible_positions[pc].imp_rot_y;
	z = bone_data[BoneIndex].possible_positions[pc].imp_rot_z;

}
void Hand::GetJointPos(int jn, int &pc)
{
	assert(bone_map.find(joints[jn]) != bone_map.end());
	int BoneIndex = bone_map[joints[jn]];
	pc = (int)bone_data[BoneIndex].possible_positions.size();
}
void Hand::GetHandRotations(std::vector<float> &out_rx, std::vector<float> &out_ry, std::vector<float> &out_rz)
{
	out_rx = hand_pose_rotations.rot_x;
	out_ry = hand_pose_rotations.rot_y;
	out_rz = hand_pose_rotations.rot_z;
}
void Hand::SwitchAnimationStatic(bool isAnimation)
{
	param.general_animation_on = isAnimation;
}
bool Hand::CheckForRules(std::vector<int> combination)
{
	if (rules.size() == 0)
		return true;

	for (int i = 0; i < combination.size(); i++)
		assert(bone_map.find(joints[i]) != bone_map.end());

	for (int i = 0; i < combination.size(); i++)
	{
		int index_from = bone_map[joints[i]];

		for (int j = 0; j < rules.size(); j++)
		{
			if (rules[j].bone.bone_id == index_from && rules[j].bone.bone_pos == combination[i])
			{
				for (int h = 0; h < combination.size(); h++)
				{
					if (h != i)
					{
						int index_to = bone_map[joints[h]];

						bool res_with = false;
						bool help_with = false;
						for (int k = 0; k < rules[j].with.size(); k++)
						{
							if (rules[j].with[k].bone_id == index_to)
							{
								help_with = true;
								if (rules[j].with[k].bone_pos == combination[h])
									res_with = true;
							}
						}

						if (!res_with && help_with)
							return false;

						for (int k = 0; k < rules[j].without.size(); k++)
						{
							if (rules[j].without[k].bone_id == index_to && rules[j].without[k].bone_pos == combination[h])
								return false;
						}
					}
				}
			}
		}
	}

	return true;
}
void Hand::SaveProperties()
{
	std::ofstream o;
	o.open(param.setup_model_property_path);
	assert(o.is_open());

	std::string name;
	float ox, oy, oz;
	for (int i = 0; i < hand_pose_rotations.rot_x.size(); i++)
		o << "rotationx" << " " << hand_pose_rotations.rot_x[i] << " " << 0.0f << " " << 0.0f << "\n";
	for (int i = 0; i < hand_pose_rotations.rot_y.size(); i++)
		o << "rotationy" << " " << 0.0f << " " << hand_pose_rotations.rot_y[i] << " " << 0.0f << "\n";
	for (int i = 0; i < hand_pose_rotations.rot_z.size(); i++)
		o << "rotationz" << " " << 0.0f << " " << 0.0f << " " << hand_pose_rotations.rot_z[i] << "\n";

	for (int i = 0; i < bone_data.size(); i++)
	{
		name = bone_data[i].name;
		for (int j = 0; j < bone_data[i].possible_positions.size(); j++)
		{
			ox = bone_data[i].possible_positions[j].imp_rot_x;
			oy = bone_data[i].possible_positions[j].imp_rot_y;
			oz = bone_data[i].possible_positions[j].imp_rot_z;
			o << name << " " << ox << " " << oy << " " << oz << "\n";
		}
	}
}
std::string Hand::GetJointName(int jn)
{
	return joints[jn];
}
