#ifndef __HANDPARAMETER_H__
#define __HANDPARAMETER_H__

const float boundary_max[2][26] = {
	{ -100, -100, 0,
	-360, -360, -360,

	-30, -70, -90, -90,  //0: stretch
	-90, -20, -90, -90,
	-90, -16, -90, -90,
	-90, -10, -90, -90,
	-90, -20, -90, -90 },

	{ 100, 100, 300,
	360, 360, 360,

	50, 20, 0, 0,
	0, 0, 0, 0,		//90: bend
	0, 5, 0, 0,
	0, 10, 0, 0,
	0, 20, 0, 0 }
};

struct HandParameters
{
	bool general_animation_on;
	bool general_use_rules;

	bool render_use_full_model;
	bool render_bone_sight;
	float render_FOV;

	float render_near;
	float render_far;
	float cx;
	float cy;
	float fx;
	float fy;
	float render_animationSpeed;
	int width_tile;
	int height_tile;
	int width;
	int height;
	int particle_numx;
	int particle_numy;
	int handParamNum;
	int num_joints;
	//float boundary_max[2][26];

	char* setup_vertexShaderPath;
	char* setup_fragmentShaderPath;
	char* setup_shader_modelViewProjUniformName;
	char* setup_shader_modelViewUniformName;
	char* setup_shader_textureUniformName;
	char* setup_shader_boneUniformName;
	char* setup_model_path_full;
	char* setup_model_path_low;
	char* setup_modelTexture_path;
	char* setup_model_property_path;
	char* setup_model_rules_path;
	char* setup_output_folder_path;
	

	static HandParameters Default()
	{
		HandParameters hp;

		hp.general_animation_on = true;// false;// true;
		hp.general_use_rules = true;// false;

		hp.render_use_full_model = false;
		hp.render_bone_sight = false;// true;
		
		hp.render_near = 0.01f;
		hp.render_far = 1000;//20.0f;
		hp.cx =  320.0;
		hp.cy =  240.0;
		hp.fx =  477.9;
		hp.fy =  477.9;
		
		hp.render_animationSpeed = 0.01f;
		hp.width_tile = 128;
		hp.height_tile = 128;
		hp.width = 640;
		hp.height = 480;
		hp.particle_numx = 8;
		hp.particle_numy = 4;
		hp.handParamNum = 26;
		hp.num_joints = 15;

		hp.render_FOV = 2 * atan(hp.width / (2 * hp.fx))*180.0 / 3.14;
		//hp.render_FOVx = 2 * atan(hp.width / (2 * hp.fx))*180.0 / 3.14;
		//hp.render_FOVy = 2 * atan(hp.height / (2 * hp.fy))*180.0 / 3.14;

		/*
		hp.boundary_max[0][0] = -100;	hp.boundary_max[1][0] = 100;
		hp.boundary_max[0][1] = -100;	hp.boundary_max[1][1] = 100;
		hp.boundary_max[0][2] = -100;	hp.boundary_max[1][2] = 400;
		hp.boundary_max[0][3] = -180;	hp.boundary_max[1][3] = 180;
		hp.boundary_max[0][4] = -180;	hp.boundary_max[1][4] = 180;
		hp.boundary_max[0][5] = -180;	hp.boundary_max[1][5] = 180;

		hp.boundary_max[0][6] = -90;	hp.boundary_max[1][6] = 90;
		hp.boundary_max[0][7] = -90;	hp.boundary_max[1][7] = 90;
		hp.boundary_max[0][8] = -90;	hp.boundary_max[1][8] = 90;
		hp.boundary_max[0][9] = -90;	hp.boundary_max[1][9] = 90;

		hp.boundary_max[0][10] = -90;	hp.boundary_max[1][10] = 90;
		hp.boundary_max[0][11] = -90;	hp.boundary_max[1][11] = 90;
		hp.boundary_max[0][12] = -90;	hp.boundary_max[1][12] = 90;
		hp.boundary_max[0][13] = -90;	hp.boundary_max[1][13] = 90;

		hp.boundary_max[0][14] = -90;	hp.boundary_max[1][14] = 90;
		hp.boundary_max[0][15] = -90;	hp.boundary_max[1][15] = 90;
		hp.boundary_max[0][16] = -90;	hp.boundary_max[1][16] = 90;
		hp.boundary_max[0][17] = -90;	hp.boundary_max[1][17] = 90;

		hp.boundary_max[0][18] = -90;	hp.boundary_max[1][18] = 90;
		hp.boundary_max[0][19] = -90;	hp.boundary_max[1][19] = 90;
		hp.boundary_max[0][20] = -90;	hp.boundary_max[1][20] = 90;
		hp.boundary_max[0][21] = -90;	hp.boundary_max[1][21] = 90;

		hp.boundary_max[0][22] = -90;	hp.boundary_max[1][22] = 90;
		hp.boundary_max[0][23] = -90;	hp.boundary_max[1][23] = 90;
		hp.boundary_max[0][24] = -90;	hp.boundary_max[1][24] = 90;
		hp.boundary_max[0][25] = -90;	hp.boundary_max[1][25] = 90;
		*/



		
		hp.setup_vertexShaderPath = "data/shader.vert";
		hp.setup_fragmentShaderPath = "data/shader.frag";
		//hp.setup_vertexShaderPath = "data/shader_depth.vert";
		//hp.setup_fragmentShaderPath = "data/shader_depth.frag";

		hp.setup_shader_modelViewProjUniformName = "model_view_proj_mat";
		hp.setup_shader_modelViewUniformName = "model_view_mat";
		hp.setup_shader_textureUniformName = "textureObject";
		hp.setup_shader_boneUniformName = "bone_matrix";

		hp.setup_model_path_full = "data/hand_model_full.dae";


		//hp.setup_model_path_low = "data/wristhand20180203_1.dae";
		//hp.setup_modelTexture_path = "data/wristhand20180203_1.bmp";

		hp.setup_model_path_low = "data/hand20180203_1.dae";
		hp.setup_modelTexture_path = "data/hand20180203_1.bmp";


		hp.setup_model_property_path = "data/hand.property";
		hp.setup_model_rules_path = "data/hand.property.rules";
		hp.setup_output_folder_path = "data/";

		return hp;
	}
	void CopyTo(HandParameters * dst)
	{
		dst->general_animation_on = general_animation_on;
		dst->general_use_rules = general_use_rules;

		dst->render_use_full_model = render_use_full_model;
		dst->render_bone_sight = render_bone_sight;
		dst->render_FOV = render_FOV;
		//dst->render_FOVx = render_FOVx;
		//dst->render_FOVy = render_FOVy;
		dst->render_near = render_near;
		dst->render_far = render_far;
		dst->cx = cx;
		dst->cy = cy;
		dst->fx = fx;
		dst->fy = fy;
		dst->render_animationSpeed = render_animationSpeed;

		dst->width_tile = width_tile;
		dst->height_tile = height_tile;
		dst->width = width;
		dst->height = height;
		dst->particle_numx = particle_numx;
		dst->particle_numy = particle_numy;
		dst->handParamNum = handParamNum;
		dst->num_joints = num_joints;


		//for (int i = 0; i < handParamNum; i++){
		//	dst->boundary_max[0][i] = boundary_max[0][i];
		//	dst->boundary_max[1][i] = boundary_max[1][i];
		//}

		dst->setup_vertexShaderPath = setup_vertexShaderPath;
		dst->setup_fragmentShaderPath = setup_fragmentShaderPath;
		dst->setup_shader_modelViewProjUniformName = setup_shader_modelViewProjUniformName;
		dst->setup_shader_modelViewUniformName = setup_shader_modelViewUniformName;
		dst->setup_shader_textureUniformName = setup_shader_textureUniformName;
		dst->setup_shader_boneUniformName = setup_shader_boneUniformName;

		dst->setup_model_path_full = setup_model_path_full;
		dst->setup_model_path_low = setup_model_path_low;
		dst->setup_model_property_path = setup_model_property_path;
		dst->setup_model_rules_path = setup_model_rules_path;
		dst->setup_modelTexture_path = setup_modelTexture_path;

		dst->setup_output_folder_path = setup_output_folder_path;
	}
};

#endif