#include "stdafx.h"
#include "include/SADXModLoader.h"

//SONIC_OBJECTS[0] = Sonic_Root
//SONIC_OBJECTS[1] = Sonic_Bicep_Right
//SONIC_OBJECTS[2] = Sonic_Elbow_Right
//SONIC_OBJECTS[3] = Sonic_Wrist_Right
//SONIC_OBJECTS[4] = Sonic_Palm_Right
//SONIC_OBJECTS[5] = Sonic_Fingers_Right
//SONIC_OBJECTS[6] = Sonic_Thumb_Right
//SONIC_OBJECTS[7] = Sonic_Bicep_Left
//SONIC_OBJECTS[8] = Sonic_Elbow_Left
//SONIC_OBJECTS[9] = Sonic_Wrist_Left
//SONIC_OBJECTS[10] = Sonic_Palm_Left
//SONIC_OBJECTS[11] = Sonic_Fingers_Left
//SONIC_OBJECTS[12] = Sonic_Thigh_Right
//SONIC_OBJECTS[13] = Sonic_Knee_Right
//SONIC_OBJECTS[14] = Sonic_Ankle_Right
//SONIC_OBJECTS[15] = Sonic_Heel_Right
//SONIC_OBJECTS[16] = Sonic_Toe_Right
//SONIC_OBJECTS[17] = Sonic_Thigh_Left
//SONIC_OBJECTS[18] = Sonic_Knee_Left
//SONIC_OBJECTS[19] = Sonic_Ankle_Left
//SONIC_OBJECTS[20] = Sonic_Heel_Left
//SONIC_OBJECTS[21] = Sonic_Toe_Left
//SONIC_OBJECTS[22] = SuperSonic_Root
//SONIC_OBJECTS[23] = SuperSonic_Bicep_Right
//SONIC_OBJECTS[24] = SuperSonic_Elbow_Right
//SONIC_OBJECTS[25] = SuperSonic_Wrist_Right
//SONIC_OBJECTS[26] = SuperSonic_Palm_Right
//SONIC_OBJECTS[27] = SuperSonic_Fingers_Right
//SONIC_OBJECTS[28] = SuperSonic_Bicep_Left
//SONIC_OBJECTS[29] = SuperSonic_Elbow_Left
//SONIC_OBJECTS[30] = SuperSonic_Wrist_Left
//SONIC_OBJECTS[31] = SuperSonic_Palm_Left
//SONIC_OBJECTS[32] = SuperSonic_Fingers_Left
//SONIC_OBJECTS[33] = SuperSonic_Thigh_Right
//SONIC_OBJECTS[34] = SuperSonic_Knee_Right
//SONIC_OBJECTS[35] = SuperSonic_Ankle_Right
//SONIC_OBJECTS[36] = SuperSonic_Heel_Right
//SONIC_OBJECTS[37] = SuperSonic_Toe_Right
//SONIC_OBJECTS[38] = SuperSonic_Thigh_Left
//SONIC_OBJECTS[39] = SuperSonic_Knee_Left
//SONIC_OBJECTS[40] = SuperSonic_Ankle_Left
//SONIC_OBJECTS[41] = SuperSonic_Heel_Left
//SONIC_OBJECTS[42] = SuperSonic_Toe_Left

//SONIC_OBJECTS[44] = Sonic_Jumpball
//SONIC_OBJECTS[45] = Sonic_EyeParent_Left
//SONIC_OBJECTS[46] = Sonic_Head
//SONIC_OBJECTS[47] = Sonic_Event_Head
//SONIC_OBJECTS[48] = Sonic_HeadParent_2
//SONIC_OBJECTS[49] = Sonic_Head
//SONIC_OBJECTS[50] = Sonic_HeadParent_1
//SONIC_OBJECTS[51] = Sonic_Eye_Right
//SONIC_OBJECTS[52] = Sonic_Eye_Left

//SONIC_OBJECTS[54] = LightSpeedDash_Model_Root
//SONIC_OBJECTS[55] = LightSpeedDash_Model_HeadParent_1 (Matches SONIC_OBJECTS[50])
//SONIC_OBJECTS[56] = HomingAttackSphere
//SONIC_OBJECTS[57] = HomingAttackTrail
//SONIC_OBJECTS[58] = LightSpeedDash_Shoes_Heel_Left
//SONIC_OBJECTS[59] = LightSpeedDash_Shoes_Toe_Left
//SONIC_OBJECTS[60] = LightSpeedDash_Shoes_Heel_Right
//SONIC_OBJECTS[61] = LightSpeedDash_Shoes_Toe_Right
//SONIC_OBJECTS[62] = Sonic_Wrist_Parent_Right
//SONIC_OBJECTS[63] = CrystalRing_Wrist
//SONIC_OBJECTS[64] = CrystalRing_Upgrade
//SONIC_OBJECTS[65] = LightSpeedDash_Shoes_Upgrade
//SONIC_OBJECTS[66] = Sonic_Jump_Root
//SONIC_OBJECTS[67] = Sonic_Jumpball
//SONIC_OBJECTS[68] = MetalSonic_Root
//SONIC_OBJECTS[69] = MetalSonic_Jump_Root
//SONIC_OBJECTS[70] = MetalSonic_Jumpball
//SONIC_OBJECTS[71] = Sonic_Snowboard
//SONIC_OBJECTS[72] = Spindash_Charging
//SONIC_OBJECTS[73] = LightSpeedDash_Charging_Particles
//SONIC_OBJECTS[74] = SuperSonic_Aura
//SONIC_OBJECTS[75] = SuperSonic_Moving_Charge_1
//SONIC_OBJECTS[76] = SuperSonic_Moving_Charge_2
//SONIC_OBJECTS[77] = SuperSonic_WaterEffect_1
//SONIC_OBJECTS[78] = SuperSonic_WaterEffect_2

uint16_t Sonic_UpperArmIndices[] = {
	0, 3,
	2, 5,
	4, 7,
	6, 1,
};

uint16_t Sonic_LowerArmIndices[] = {
	4, 15,
	6, 21,
	0, 19,
	2, 17,
};

uint16_t Sonic_UpperLegIndices[] = {
	5, 5,
	3, 7,
	1, 1,
	7, 3,
};

uint16_t Sonic_LowerLegIndices[] = {
	6, 5,
	0, 7,
	2, 1,
	4, 3,
};

uint16_t Sonic_RightShoeIndices[] = {
	17, 1,
	21, 11,
	20, 7,
	19, 6,
	18, 12,
	16, 5,
};

uint16_t Sonic_RightLSShoeIndices[] = {
	0, 1,
	25, 11,
	15, 7,
	14, 6,
	24, 12,
	1, 5,
};

uint16_t Sonic_RightHandIndices[] = {
	0, 15,
	6, 14,
	7, 13,
	1, 12,
};

uint16_t Sonic_LeftShoeIndices[] = {
	12, 5,
	20, 6,
	21, 13,
	19, 10,
	18, 12,
	13, 1,
};

uint16_t Sonic_LeftLSShoeIndices[] = {
	0, 5,
	12, 6,
	14, 13,
	10, 10,
	9, 12,
	1, 1,
};

uint16_t Sonic_LeftHandIndices[] = {
	8, 14,
	4, 15,
	3, 12,
	2, 13,
};

void __cdecl InitSonicWeldInfo_mod()
{
	NJS_OBJECT *v0; // ebp@1
	NJS_OBJECT *v1; // ebp@1
	NJS_OBJECT *v2; // ebp@1
	NJS_OBJECT *v3; // ebp@1
	NJS_OBJECT *v4; // edi@1
	NJS_OBJECT *v5; // eax@1

					//The following welds are for Sonic
	SonicWeldInfo[0].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[0].ModelA = SONIC_OBJECTS[1];
	SonicWeldInfo[0].ModelB = SONIC_OBJECTS[2];
	SonicWeldInfo[0].anonymous_5 = 0;
	SonicWeldInfo[0].VertexBuffer = 0;
	SonicWeldInfo[0].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperArmIndices) / 2);
	SonicWeldInfo[0].WeldType = 2;
	SonicWeldInfo[0].VertIndexes = Sonic_UpperArmIndices;
	SonicWeldInfo[1].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[1].ModelA = SONIC_OBJECTS[2];
	SonicWeldInfo[1].ModelB = SONIC_OBJECTS[3];
	SonicWeldInfo[1].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerArmIndices) / 2);
	SonicWeldInfo[1].WeldType = 2;
	SonicWeldInfo[1].anonymous_5 = 0;
	SonicWeldInfo[1].VertexBuffer = 0;
	SonicWeldInfo[1].VertIndexes = Sonic_LowerArmIndices;
	SonicWeldInfo[2].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[2].ModelA = SONIC_OBJECTS[7];
	SonicWeldInfo[2].ModelB = SONIC_OBJECTS[8];
	SonicWeldInfo[2].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperArmIndices) / 2);
	SonicWeldInfo[2].WeldType = 2;
	SonicWeldInfo[2].anonymous_5 = 0;
	SonicWeldInfo[2].VertexBuffer = 0;
	SonicWeldInfo[2].VertIndexes = Sonic_UpperArmIndices;
	SonicWeldInfo[3].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[3].ModelA = SONIC_OBJECTS[8];
	v0 = SONIC_OBJECTS[9];
	SonicWeldInfo[3].VertIndexes = Sonic_LowerArmIndices;
	SonicWeldInfo[3].ModelB = v0;
	SonicWeldInfo[3].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerArmIndices) / 2);
	SonicWeldInfo[3].WeldType = 2;
	SonicWeldInfo[3].anonymous_5 = 0;
	SonicWeldInfo[3].VertexBuffer = 0;
	SonicWeldInfo[4].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[4].ModelA = SONIC_OBJECTS[12];
	SonicWeldInfo[4].ModelB = SONIC_OBJECTS[13];
	SonicWeldInfo[4].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperLegIndices) / 2);
	SonicWeldInfo[4].WeldType = 2;
	SonicWeldInfo[4].anonymous_5 = 0;
	SonicWeldInfo[4].VertexBuffer = 0;
	SonicWeldInfo[4].VertIndexes = Sonic_UpperLegIndices;
	SonicWeldInfo[5].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[5].ModelA = SONIC_OBJECTS[13];
	SonicWeldInfo[5].ModelB = SONIC_OBJECTS[14];
	SonicWeldInfo[5].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerLegIndices) / 2);
	SonicWeldInfo[5].WeldType = 2;
	SonicWeldInfo[5].anonymous_5 = 0;
	SonicWeldInfo[5].VertexBuffer = 0;
	SonicWeldInfo[5].VertIndexes = Sonic_LowerLegIndices;
	SonicWeldInfo[6].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[6].ModelA = SONIC_OBJECTS[17];
	v1 = SONIC_OBJECTS[18];
	SonicWeldInfo[6].VertIndexes = Sonic_UpperLegIndices;
	SonicWeldInfo[6].ModelB = v1;
	SonicWeldInfo[6].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperLegIndices) / 2);
	SonicWeldInfo[6].WeldType = 2;
	SonicWeldInfo[6].anonymous_5 = 0;
	SonicWeldInfo[6].VertexBuffer = 0;
	SonicWeldInfo[7].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[7].ModelA = SONIC_OBJECTS[18];
	SonicWeldInfo[7].ModelB = SONIC_OBJECTS[19];
	SonicWeldInfo[7].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerLegIndices) / 2);
	SonicWeldInfo[7].WeldType = 2;
	SonicWeldInfo[7].anonymous_5 = 0;
	SonicWeldInfo[7].VertexBuffer = 0;
	SonicWeldInfo[7].VertIndexes = Sonic_LowerLegIndices;
	SonicWeldInfo[8].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[8].ModelA = SONIC_OBJECTS[15];
	SonicWeldInfo[8].ModelB = SONIC_OBJECTS[16];
	SonicWeldInfo[8].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_RightShoeIndices) / 2);
	SonicWeldInfo[8].WeldType = 2;
	SonicWeldInfo[8].anonymous_5 = 0;
	SonicWeldInfo[8].VertexBuffer = 0;
	SonicWeldInfo[8].VertIndexes = Sonic_RightShoeIndices;
	SonicWeldInfo[9].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[9].ModelA = SONIC_OBJECTS[20];
	v2 = SONIC_OBJECTS[21];
	SonicWeldInfo[9].VertIndexes = Sonic_LeftShoeIndices;
	SonicWeldInfo[9].ModelB = v2;
	SonicWeldInfo[9].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LeftShoeIndices) / 2);
	SonicWeldInfo[9].WeldType = 2;
	SonicWeldInfo[9].anonymous_5 = 0;
	SonicWeldInfo[9].VertexBuffer = 0;
	SonicWeldInfo[10].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[10].ModelA = SONIC_OBJECTS[10];
	SonicWeldInfo[10].ModelB = SONIC_OBJECTS[11];
	SonicWeldInfo[10].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LeftHandIndices) / 2);
	SonicWeldInfo[10].WeldType = 2;
	SonicWeldInfo[10].anonymous_5 = 0;
	SonicWeldInfo[10].VertexBuffer = 0;
	SonicWeldInfo[10].VertIndexes = Sonic_LeftHandIndices;
	SonicWeldInfo[11].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[11].ModelA = SONIC_OBJECTS[4];
	SonicWeldInfo[11].ModelB = SONIC_OBJECTS[5];
	SonicWeldInfo[11].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_RightHandIndices) / 2);
	SonicWeldInfo[11].WeldType = 2;
	SonicWeldInfo[11].anonymous_5 = 0;
	SonicWeldInfo[11].VertexBuffer = 0;
	SonicWeldInfo[11].VertIndexes = Sonic_RightHandIndices;
	SonicWeldInfo[12].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[12].ModelA = SONIC_OBJECTS[58];
	SonicWeldInfo[12].ModelB = SONIC_OBJECTS[59];
	SonicWeldInfo[12].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LeftLSShoeIndices) / 2);
	SonicWeldInfo[12].WeldType = 2;
	SonicWeldInfo[12].anonymous_5 = 0;
	SonicWeldInfo[12].VertexBuffer = 0;
	SonicWeldInfo[12].VertIndexes = Sonic_LeftLSShoeIndices;
	SonicWeldInfo[13].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[13].ModelA = SONIC_OBJECTS[60];
	SonicWeldInfo[13].ModelB = SONIC_OBJECTS[61];
	SonicWeldInfo[13].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_RightLSShoeIndices) / 2);
	SonicWeldInfo[13].WeldType = 2;
	SonicWeldInfo[13].anonymous_5 = 0;
	SonicWeldInfo[13].VertexBuffer = 0;
	SonicWeldInfo[13].VertIndexes = Sonic_RightLSShoeIndices;
	SonicWeldInfo[14].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[14].ModelA = SONIC_OBJECTS[2];
	SonicWeldInfo[14].ModelB = SONIC_OBJECTS[63];
	SonicWeldInfo[14].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerArmIndices) / 2);
	SonicWeldInfo[14].WeldType = 2;
	SonicWeldInfo[14].anonymous_5 = 0;
	SonicWeldInfo[14].VertexBuffer = 0;
	SonicWeldInfo[14].VertIndexes = Sonic_LowerArmIndices;
	SonicWeldInfo[15].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[15].ModelA = SONIC_OBJECTS[5];
	SonicWeldInfo[15].ModelB = 0;
	SonicWeldInfo[15].VertexPairCount = 2;
	SonicWeldInfo[15].WeldType = 4;
	SonicWeldInfo[15].anonymous_5 = 0;
	SonicWeldInfo[15].VertexBuffer = 0;
	SonicWeldInfo[15].VertIndexes = 0;
	SonicWeldInfo[16].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[16].ModelA = SONIC_OBJECTS[11];
	SonicWeldInfo[16].ModelB = 0;
	SonicWeldInfo[16].VertexPairCount = 2;
	SonicWeldInfo[16].WeldType = 5;
	SonicWeldInfo[16].anonymous_5 = 0;
	SonicWeldInfo[16].VertexBuffer = 0;
	SonicWeldInfo[16].VertIndexes = 0;
	SonicWeldInfo[17].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[17].ModelA = SONIC_OBJECTS[59];
	SonicWeldInfo[17].ModelB = 0;
	SonicWeldInfo[17].VertexPairCount = 0;
	SonicWeldInfo[17].WeldType = 7;
	SonicWeldInfo[17].anonymous_5 = 0;
	SonicWeldInfo[17].VertexBuffer = 0;
	SonicWeldInfo[17].VertIndexes = 0;
	SonicWeldInfo[18].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[18].ModelA = SONIC_OBJECTS[61];
	SonicWeldInfo[18].ModelB = 0;
	SonicWeldInfo[18].VertexPairCount = 0;
	SonicWeldInfo[18].WeldType = 6;
	SonicWeldInfo[18].anonymous_5 = 0;
	SonicWeldInfo[18].VertexBuffer = 0;
	SonicWeldInfo[18].VertIndexes = 0;
	SonicWeldInfo[19].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[19].ModelA = SONIC_OBJECTS[16];
	SonicWeldInfo[19].ModelB = 0;
	SonicWeldInfo[19].VertexPairCount = 0;
	SonicWeldInfo[19].WeldType = 6;
	SonicWeldInfo[19].anonymous_5 = 0;
	SonicWeldInfo[19].VertexBuffer = 0;
	SonicWeldInfo[19].VertIndexes = 0;
	SonicWeldInfo[20].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[20].ModelA = SONIC_OBJECTS[21];
	SonicWeldInfo[20].ModelB = 0;
	SonicWeldInfo[20].VertexPairCount = 0;
	SonicWeldInfo[20].WeldType = 7;
	SonicWeldInfo[20].anonymous_5 = 0;
	SonicWeldInfo[20].VertexBuffer = 0;
	SonicWeldInfo[20].VertIndexes = 0;
	SonicWeldInfo[21].BaseModel = SONIC_OBJECTS[0];
	SonicWeldInfo[21].ModelA = SONIC_OBJECTS[45];
	SonicWeldInfo[21].ModelB = 0;
	SonicWeldInfo[21].VertexPairCount = 0;
	SonicWeldInfo[21].WeldType = 8;
	SonicWeldInfo[21].anonymous_5 = 0;
	SonicWeldInfo[21].VertexBuffer = 0;
	SonicWeldInfo[21].VertIndexes = 0;

	//The following welds are for Super Sonic
	SonicWeldInfo[22].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[22].ModelA = SONIC_OBJECTS[23];
	SonicWeldInfo[22].ModelB = SONIC_OBJECTS[24];
	SonicWeldInfo[22].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperArmIndices) / 2);
	SonicWeldInfo[22].WeldType = 2;
	SonicWeldInfo[22].anonymous_5 = 0;
	SonicWeldInfo[22].VertexBuffer = 0;
	SonicWeldInfo[22].VertIndexes = Sonic_UpperArmIndices;
	SonicWeldInfo[23].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[23].ModelA = SONIC_OBJECTS[24];
	SonicWeldInfo[23].ModelB = SONIC_OBJECTS[25];
	SonicWeldInfo[23].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerArmIndices) / 2);
	SonicWeldInfo[23].WeldType = 2;
	SonicWeldInfo[23].anonymous_5 = 0;
	SonicWeldInfo[23].VertexBuffer = 0;
	SonicWeldInfo[23].VertIndexes = Sonic_LowerArmIndices;
	SonicWeldInfo[24].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[24].ModelA = SONIC_OBJECTS[28];
	SonicWeldInfo[24].ModelB = SONIC_OBJECTS[29];
	SonicWeldInfo[24].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperArmIndices) / 2);
	SonicWeldInfo[24].WeldType = 2;
	SonicWeldInfo[24].anonymous_5 = 0;
	SonicWeldInfo[24].VertexBuffer = 0;
	SonicWeldInfo[24].VertIndexes = Sonic_UpperArmIndices;
	SonicWeldInfo[25].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[25].ModelA = SONIC_OBJECTS[29];
	SonicWeldInfo[25].ModelB = SONIC_OBJECTS[30];
	SonicWeldInfo[25].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerArmIndices) / 2);
	SonicWeldInfo[25].WeldType = 2;
	SonicWeldInfo[25].anonymous_5 = 0;
	SonicWeldInfo[25].VertexBuffer = 0;
	SonicWeldInfo[25].VertIndexes = Sonic_LowerArmIndices;
	SonicWeldInfo[26].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[26].ModelA = SONIC_OBJECTS[33];
	SonicWeldInfo[26].ModelB = SONIC_OBJECTS[34];
	SonicWeldInfo[26].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperLegIndices) / 2);
	SonicWeldInfo[26].WeldType = 2;
	SonicWeldInfo[26].anonymous_5 = 0;
	SonicWeldInfo[26].VertexBuffer = 0;
	SonicWeldInfo[26].VertIndexes = Sonic_UpperLegIndices;
	SonicWeldInfo[27].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[27].ModelA = SONIC_OBJECTS[34];
	SonicWeldInfo[27].ModelB = SONIC_OBJECTS[35];
	SonicWeldInfo[27].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerLegIndices) / 2);
	SonicWeldInfo[27].WeldType = 2;
	SonicWeldInfo[27].anonymous_5 = 0;
	SonicWeldInfo[27].VertexBuffer = 0;
	SonicWeldInfo[27].VertIndexes = Sonic_LowerLegIndices;
	SonicWeldInfo[28].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[28].ModelA = SONIC_OBJECTS[38];
	v3 = SONIC_OBJECTS[39];
	SonicWeldInfo[28].VertIndexes = Sonic_UpperLegIndices;
	SonicWeldInfo[28].ModelB = v3;
	SonicWeldInfo[28].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_UpperLegIndices) / 2);
	SonicWeldInfo[28].WeldType = 2;
	SonicWeldInfo[28].anonymous_5 = 0;
	SonicWeldInfo[28].VertexBuffer = 0;
	SonicWeldInfo[29].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[29].ModelA = SONIC_OBJECTS[39];
	SonicWeldInfo[29].ModelB = SONIC_OBJECTS[40];
	SonicWeldInfo[29].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LowerLegIndices) / 2);
	SonicWeldInfo[29].WeldType = 2;
	SonicWeldInfo[29].anonymous_5 = 0;
	SonicWeldInfo[29].VertexBuffer = 0;
	SonicWeldInfo[29].VertIndexes = Sonic_LowerLegIndices;
	SonicWeldInfo[30].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[30].ModelA = SONIC_OBJECTS[36];
	SonicWeldInfo[30].ModelB = SONIC_OBJECTS[37];
	SonicWeldInfo[30].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_RightLSShoeIndices) / 2);
	SonicWeldInfo[30].WeldType = 2;
	SonicWeldInfo[30].anonymous_5 = 0;
	SonicWeldInfo[30].VertexBuffer = 0;
	SonicWeldInfo[30].VertIndexes = Sonic_RightLSShoeIndices;
	SonicWeldInfo[31].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[31].ModelA = SONIC_OBJECTS[41];
	SonicWeldInfo[31].ModelB = SONIC_OBJECTS[42];
	SonicWeldInfo[31].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LeftLSShoeIndices) / 2);
	SonicWeldInfo[31].WeldType = 2;
	SonicWeldInfo[31].anonymous_5 = 0;
	SonicWeldInfo[31].VertexBuffer = 0;
	SonicWeldInfo[31].VertIndexes = Sonic_LeftLSShoeIndices;
	SonicWeldInfo[32].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[32].ModelA = SONIC_OBJECTS[31];
	SonicWeldInfo[32].ModelB = SONIC_OBJECTS[32];
	SonicWeldInfo[32].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_LeftHandIndices) / 2);
	SonicWeldInfo[32].WeldType = 2;
	SonicWeldInfo[32].anonymous_5 = 0;
	SonicWeldInfo[32].VertexBuffer = 0;
	SonicWeldInfo[32].VertIndexes = Sonic_LeftHandIndices;
	SonicWeldInfo[33].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[33].ModelA = SONIC_OBJECTS[26];
	v4 = SONIC_OBJECTS[27];
	SonicWeldInfo[33].anonymous_5 = 0;
	SonicWeldInfo[33].VertexBuffer = 0;
	SonicWeldInfo[33].VertIndexes = Sonic_RightHandIndices;
	SonicWeldInfo[33].ModelB = v4;
	SonicWeldInfo[33].VertexPairCount = (uint8_t)(LengthOfArray(Sonic_RightHandIndices) / 2);
	SonicWeldInfo[33].WeldType = 2;
	SonicWeldInfo[34].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[34].ModelA = SONIC_OBJECTS[31];
	SonicWeldInfo[34].ModelB = SONIC_OBJECTS[32];
	SonicWeldInfo[34].anonymous_5 = 0;
	SonicWeldInfo[34].VertexBuffer = 0;
	SonicWeldInfo[34].VertexPairCount = 4;
	SonicWeldInfo[34].VertIndexes = Sonic_RightHandIndices;
	SonicWeldInfo[34].WeldType = 2;
	SonicWeldInfo[35].BaseModel = SONIC_OBJECTS[22];
	SonicWeldInfo[35].ModelA = SONIC_OBJECTS[26];
	v5 = SONIC_OBJECTS[27];
	SonicWeldInfo[35].anonymous_5 = 0;
	SonicWeldInfo[35].VertexBuffer = 0;
	SonicWeldInfo[36].BaseModel = 0;
	SonicWeldInfo[36].ModelA = 0;
	SonicWeldInfo[36].ModelB = 0;
	SonicWeldInfo[35].VertIndexes = Sonic_RightHandIndices;
	SonicWeldInfo[36].VertexPairCount = 0;
	SonicWeldInfo[36].VertexBuffer = 0;
	SonicWeldInfo[35].VertexPairCount = 4;
	SonicWeldInfo[35].ModelB = v5;
	SonicWeldInfo[35].WeldType = 2;
	SonicWeldInfo[36].VertIndexes = 0;
}