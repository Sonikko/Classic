#include "stdafx.h"
#include "include/SADXModLoader.h"
#include "Sonic.h"
#include "SuperSonic.h"
#include "Miles.h"
#include "Knuckles.h"
#include "Functions.h"
#include "math.h"

#define ReplacePVM(a)  helperFunctions.ReplaceFile("system\\" a ".PVM", "system\\" a "_Classic.PVM");
//DataArray(WeldInfo, NPCKnucklesWeldInfo, 0x03C53038, 0x19);

//Sonic Morph Animations, by IEA, no idea what's in here
void __cdecl Sonic_MorphStretchyFeet_c(CharObj2* a1)
{
	float a4; // ST10_4@3
	float v2; // ST10_4@5
	NJS_MODEL_SADX *v3; // ecx@5
	float v4; // ST10_4@10
	float v5; // ST10_4@13

	{if (SONIC_OBJECTS[19]->sibling == SONIC_OBJECTS[58])

		if (a1->AnimationThing.Index == 13)
		{
			if (a1->AnimationThing.Frame >= 16.0f)
			{
				if (a1->AnimationThing.Frame <= 24.0f)
				{
					a1->SomeFrameNumberThing = 0.0f;
					v3 = SONIC_MODELS[4];
				}
				else
				{
					v2 = (a1->AnimationThing.Frame - 24.0f) * 0.125f;
					a1->SomeFrameNumberThing = v2;
					MorphPoints(SONIC_MODELS[4], SONIC_MODELS[3], SONIC_MODELS[2], v2);
					v3 = SONIC_MODELS[2];
				}
				SONIC_OBJECTS[59]->model = v3;
			}
			else
			{
				a4 = (16.0f - a1->AnimationThing.Frame) * 0.0625f;
				a1->SomeFrameNumberThing = a4;
				MorphPoints(SONIC_MODELS[4], SONIC_MODELS[3], SONIC_MODELS[2], a4);
				SONIC_OBJECTS[59]->model = SONIC_MODELS[2];
			}
			if (a1->AnimationThing.Frame < 8.0f || a1->AnimationThing.Frame > 16.0f)
			{
				if (a1->AnimationThing.Frame <= 16.0f || a1->AnimationThing.Frame > 32.0f)
				{
					a1->TailsFlightTime = 0.0f;
					SONIC_OBJECTS[61]->model = SONIC_MODELS[7];
				}
				else
				{
					v5 = (32.0f - a1->AnimationThing.Frame) * 0.0625f;
					a1->TailsFlightTime = v5;
					MorphPoints(SONIC_MODELS[7], SONIC_MODELS[6], SONIC_MODELS[5], v5);
					SONIC_OBJECTS[61]->model = SONIC_MODELS[5];
				}
			}
			else
			{
				v4 = (a1->AnimationThing.Frame - 8.0f) * 0.125f;
				a1->TailsFlightTime = v4;
				MorphPoints(SONIC_MODELS[7], SONIC_MODELS[6], SONIC_MODELS[5], v4);
				SONIC_OBJECTS[61]->model = SONIC_MODELS[5];
			}
		}
		else
		{
			a1->SomeFrameNumberThing = 0.0f;
			a1->TailsFlightTime = 0.0f;
			SONIC_OBJECTS[59]->model = &attach_00581880;
			SONIC_OBJECTS[61]->model = &attach_00582578;
		}

	else
	{
		if (a1->AnimationThing.Index == 13)
		{
			if (a1->AnimationThing.Frame >= 16.0f)
			{
				if (a1->AnimationThing.Frame <= 24.0f)
				{
					a1->SomeFrameNumberThing = 0.0f;
					v3 = SONIC_MODELS[4];
				}
				else
				{
					v2 = (a1->AnimationThing.Frame - 24.0f) * 0.125f;
					a1->SomeFrameNumberThing = v2;
					MorphPoints(SONIC_MODELS[4], SONIC_MODELS[3], SONIC_MODELS[2], v2);
					v3 = SONIC_MODELS[2];
				}
				SONIC_OBJECTS[21]->model = v3;
			}
			else
			{
				a4 = (16.0f - a1->AnimationThing.Frame) * 0.0625f;
				a1->SomeFrameNumberThing = a4;
				MorphPoints(SONIC_MODELS[4], SONIC_MODELS[3], SONIC_MODELS[2], a4);
				SONIC_OBJECTS[21]->model = SONIC_MODELS[2];
			}
			if (a1->AnimationThing.Frame < 8.0f || a1->AnimationThing.Frame > 16.0f)
			{
				if (a1->AnimationThing.Frame <= 16.0f || a1->AnimationThing.Frame > 32.0f)
				{
					a1->TailsFlightTime = 0.0f;
					SONIC_OBJECTS[16]->model = SONIC_MODELS[7];
				}
				else
				{
					v5 = (32.0f - a1->AnimationThing.Frame) * 0.0625f;
					a1->TailsFlightTime = v5;
					MorphPoints(SONIC_MODELS[7], SONIC_MODELS[6], SONIC_MODELS[5], v5);
					SONIC_OBJECTS[16]->model = SONIC_MODELS[5];
				}
			}
			else
			{
				v4 = (a1->AnimationThing.Frame - 8.0f) * 0.125f;
				a1->TailsFlightTime = v4;
				MorphPoints(SONIC_MODELS[7], SONIC_MODELS[6], SONIC_MODELS[5], v4);
				SONIC_OBJECTS[16]->model = SONIC_MODELS[5];
			}
		}
		else
		{
			a1->SomeFrameNumberThing = 0.0f;
			a1->TailsFlightTime = 0.0f;
			SONIC_OBJECTS[21]->model = SONIC_MODELS[0];
			SONIC_OBJECTS[16]->model = SONIC_MODELS[1];
		}
	}
	}
}
static void __declspec(naked) Sonic_MorphStretchyFeet_asm()
{
	__asm
	{
		push esi // a1
		call Sonic_MorphStretchyFeet_c
		pop esi // a1
		retn
	}
}

//Tails Head Morphs, vertex indices +1 for whatever reason, WORKING
const Sint16 MorphVertsT[] = {
	// right ear
	42, 43, 55, 56,
	// left ear
	39, 110, 111, 109,
	// hair (top, middle, bottom)
	137, 130, 148,
	// right whiskers
	153, 157, 161, 165,
	// left whiskers
	168, 172, 176, 180
};

//Tails Jiggle Function, copied from SA1_Chars

static void __cdecl Tails_Jiggle_mod(ObjectMaster *_this)
{
	
	NJS_POINT3 *_src_points; // esi@4
	float v21; // st7@32
	Sint32 nbPoint; // [sp+14h] [bp-14h]@4
	NJS_VECTOR a2; // [sp+1Ch] [bp-Ch]@29

	EntityData2* v1 = (EntityData2*)_this->Data2;
	EntityData1* v2 = _this->Data1;
	int v3 = v2->CharIndex;

	if (!EntityData1Ptrs[v3])
	{
		CheckThingButThenDeleteObject(_this);
		return;
	}

	EntityData2 *v25 = EntityData2Ptrs[v3];
	CharObj2 *v4 = GetCharObj2(v2->CharIndex);
	NJS_POINT3 *_dst_points = MILES_MODELS[4]->points;
	NJS_OBJECT *v6 = TailsAnimData[v4->AnimationThing.Index].Animation->object;

	if (v6 == *MILES_OBJECTS)
	{
		NJS_MODEL_SADX *v7 = MILES_MODELS[2];
		_src_points = v7->points;
		nbPoint = v7->nbPoint;
	}
	else
	{
		if (v6 != MILES_OBJECTS[1])
		{
			return;
		}

		NJS_MODEL_SADX *v9 = MILES_MODELS[3];
		_src_points = v9->points;
		nbPoint = v9->nbPoint;
	}

	if (!v2->Action)
	{
		v2->Action = 1;
		_this->DeleteSub = Tails_Jiggle_Delete;
	}
	else if (v2->Action == 1)
	{
		float v10 = v25->VelocityDirection.x * v25->VelocityDirection.x
			+ v25->VelocityDirection.y * v25->VelocityDirection.y
			+ v25->VelocityDirection.z * v25->VelocityDirection.z;
		float v11 = squareroot(v10);
		float v12 = v11 + v11;

		if (fabs(v1->VelocityDirection.y) < v12)
		{
			if (v12 < 0.0f)
			{
				v12 = -v12;
			}
			if (v12 > 4.0f)
			{
				v12 = 4.0f;
			}
			v1->VelocityDirection.y = v12;
		}

		if (v1->VelocityDirection.y >= 0.2f || v1->VelocityDirection.y <= -0.2f)
		{
			v1->VelocityDirection.y = v1->VelocityDirection.y * 0.99000001f;
		}
		else
		{
			v1->VelocityDirection.y = 0.0f;
		}

		signed int v13 = (int)fabs(v1->VelocityDirection.y * 512.0f) + 2048;
		if (v13 > 3072)
		{
			v13 = 3072;
		}

		Angle v14 = v13 + v1->field_30;
		v1->field_30 = v14;
		v2->Rotation.z = (int)(v1->VelocityDirection.y * 0.2f * njCos(v14) * 4096.0f);

		njUnitMatrix(TailsMatrix1);
		Angle v15 = v2->Rotation.z;
		if (v15)
		{
			njRotateZ(TailsMatrix1, (unsigned __int16)v15);
		}

		njUnitMatrix(TailsMatrix2);
		Angle v16 = v2->Rotation.z >> 1;
		if (v16)
		{
			njRotateZ(TailsMatrix2, (unsigned __int16)v16);
		}

		if (nbPoint)
		{
			for (auto i : MorphVertsT)
			{
				auto src_points = &_src_points[i - 1];
				auto dst_points = &_dst_points[i - 1];

				if (src_points->y >= -2.9000001f)
				{
					if (src_points->y <= -1.65f || fabs(src_points->z) <= 1.15f)
					{
						if (src_points->y >= -2.0999999f || src_points->x <= 1.3f)
						{
							continue;
						}

						float v22 = src_points->x + 0.25f;
						float v23 = src_points->y;
						a2.z = src_points->z;
						a2.x = v22;
						a2.y = v23;
						njCalcVector(TailsMatrix2, &a2, dst_points);
						v21 = dst_points->x - 0.25f;
					}
					else
					{
						float v19 = src_points->z;
						float v20 = src_points->x + 1.5f;
						a2.y = src_points->y;
						a2.z = v19;
						a2.x = v20;
						njCalcVector(TailsMatrix1, &a2, dst_points);
						v21 = dst_points->x - 1.5f;
					}
					dst_points->x = v21;
				}
				else
				{
					float v18 = src_points->z;
					a2.x = src_points->x + 0.5f;
					a2.z = v18;
					a2.y = src_points->y + 1.5f;
					njCalcVector(TailsMatrix1, &a2, dst_points);
					dst_points->x = dst_points->x - 0.5f;
					dst_points->y = dst_points->y - 1.5f;
				}
			}
		}
	}
	else
	{
		DeleteObjectMaster(_this);
	}

	MILES_OBJECTS[4]->model = MILES_MODELS[4];
	MILES_OBJECTS[5]->model = MILES_MODELS[4];
}

//Models Calls
extern "C" __declspec(dllexport) void __cdecl Init(const char *path, const HelperFunctions &helperFunctions)
{
	HMODULE handle = GetModuleHandle(L"CHRMODELS_orig");
	NJS_OBJECT **___SONIC_OBJECTS = (NJS_OBJECT **)GetProcAddress(handle, "___SONIC_OBJECTS");
	NJS_ACTION **___SONIC_ACTIONS = (NJS_ACTION **)GetProcAddress(handle, "___SONIC_ACTIONS");
	NJS_MODEL_SADX **___SONIC_MODELS = (NJS_MODEL_SADX **)GetProcAddress(handle, "___SONIC_MODELS");
	
	//Sonic
	___SONIC_OBJECTS[0] = &object_0056AF50;
	___SONIC_OBJECTS[1] = &object_00563B7C;
	___SONIC_OBJECTS[2] = &object_00563D0C;
	___SONIC_OBJECTS[3] = &object_005654EC;
	___SONIC_OBJECTS[4] = &object_00564CD0;
	___SONIC_OBJECTS[5] = &object_005647B8;
	___SONIC_OBJECTS[6] = &object_00564A78;
	___SONIC_OBJECTS[7] = &object_00561F14;
	___SONIC_OBJECTS[8] = &object_005620A4;
	___SONIC_OBJECTS[9] = &object_005638CC;
	___SONIC_OBJECTS[10] = &object_005630B0;
	___SONIC_OBJECTS[11] = &object_00562B80;
	___SONIC_OBJECTS[12] = &object_0056044C;
	___SONIC_OBJECTS[13] = &object_005605DC;
	___SONIC_OBJECTS[14] = &object_00561C68;
	___SONIC_OBJECTS[15] = &object_005613F8;
	___SONIC_OBJECTS[16] = &object_00560DD0;
	___SONIC_OBJECTS[17] = &object_0055E99C;
	___SONIC_OBJECTS[18] = &object_0055EB2C;
	___SONIC_OBJECTS[19] = &object_005601B8;
	___SONIC_OBJECTS[20] = &object_0055F948;
	___SONIC_OBJECTS[21] = &object_0055F330;
	___SONIC_OBJECTS[45] = &object_0056998C;
	___SONIC_OBJECTS[46] = &object_00569594;
	___SONIC_OBJECTS[48] = &object_00569DEC;
	___SONIC_OBJECTS[49] = &object_00569594;
	___SONIC_OBJECTS[50] = &object_00569E20;
	___SONIC_OBJECTS[51] = &object_00569CE8;
	___SONIC_OBJECTS[52] = &object_005698F0;
	___SONIC_OBJECTS[54] = &object_006837E8;
	___SONIC_OBJECTS[55] = &object_00682EF4;
	___SONIC_OBJECTS[58] = &object_00581FB8;
	___SONIC_OBJECTS[59] = &object_005818AC;
	___SONIC_OBJECTS[60] = &object_00582CC0;
	___SONIC_OBJECTS[61] = &object_005825A4;
	___SONIC_OBJECTS[62] = &object_00565520;
	___SONIC_OBJECTS[63] = &object_00583284;
	___SONIC_OBJECTS[66] = &object_005729CC;
	___SONIC_ACTIONS[0]->object = &object_0056AF50;
	___SONIC_ACTIONS[1]->object = &object_0056AF50;
	___SONIC_ACTIONS[2]->object = &object_0056AF50;
	___SONIC_ACTIONS[3]->object = &object_0056AF50;
	___SONIC_ACTIONS[4]->object = &object_0056AF50;
	___SONIC_ACTIONS[5]->object = &object_0056AF50;
	___SONIC_ACTIONS[6]->object = &object_0056AF50;
	___SONIC_ACTIONS[7]->object = &object_0056AF50;
	___SONIC_ACTIONS[8]->object = &object_0056AF50;
	___SONIC_ACTIONS[9]->object = &object_0056AF50;
	___SONIC_ACTIONS[10]->object = &object_0056AF50;
	___SONIC_ACTIONS[11]->object = &object_0056AF50;
	___SONIC_ACTIONS[12]->object = &object_0056AF50;
	___SONIC_ACTIONS[13]->object = &object_0056AF50;
	___SONIC_ACTIONS[14]->object = &object_005729CC;
	___SONIC_ACTIONS[15]->object = &object_0056AF50;
	___SONIC_ACTIONS[16]->object = &object_0056AF50;
	___SONIC_ACTIONS[17]->object = &object_0056AF50;
	___SONIC_ACTIONS[18]->object = &object_0056AF50;
	___SONIC_ACTIONS[19]->object = &object_0056AF50;
	___SONIC_ACTIONS[20]->object = &object_0056AF50;
	___SONIC_ACTIONS[22]->object = &object_0056AF50;
	___SONIC_ACTIONS[23]->object = &object_0056AF50;
	___SONIC_ACTIONS[27]->object = &object_0056AF50;
	___SONIC_ACTIONS[28]->object = &object_0056AF50;
	___SONIC_ACTIONS[29]->object = &object_0056AF50;
	___SONIC_ACTIONS[30]->object = &object_0056AF50;
	___SONIC_ACTIONS[31]->object = &object_0056AF50;
	___SONIC_ACTIONS[32]->object = &object_0056AF50;
	___SONIC_ACTIONS[33]->object = &object_0056AF50;
	___SONIC_ACTIONS[34]->object = &object_0056AF50;
	___SONIC_ACTIONS[35]->object = &object_0056AF50;
	___SONIC_ACTIONS[36]->object = &object_0056AF50;
	___SONIC_ACTIONS[37]->object = &object_0056AF50;
	___SONIC_ACTIONS[38]->object = &object_0056AF50;
	___SONIC_ACTIONS[39]->object = &object_0056AF50;
	___SONIC_ACTIONS[40]->object = &object_0056AF50;
	___SONIC_ACTIONS[41]->object = &object_0056AF50;
	___SONIC_ACTIONS[42]->object = &object_0056AF50;
	___SONIC_ACTIONS[43]->object = &object_0056AF50;
	___SONIC_ACTIONS[44]->object = &object_0056AF50;
	___SONIC_ACTIONS[45]->object = &object_0056AF50;
	___SONIC_ACTIONS[46]->object = &object_0056AF50;
	___SONIC_ACTIONS[47]->object = &object_0056AF50;
	___SONIC_ACTIONS[48]->object = &object_0056AF50;
	___SONIC_ACTIONS[49]->object = &object_0056AF50;
	___SONIC_ACTIONS[50]->object = &object_0056AF50;
	___SONIC_ACTIONS[51]->object = &object_0056AF50;
	___SONIC_ACTIONS[52]->object = &object_0056AF50;
	___SONIC_ACTIONS[53]->object = &object_0056AF50;
	___SONIC_ACTIONS[54]->object = &object_0056AF50;
	___SONIC_ACTIONS[55]->object = &object_0056AF50;
	___SONIC_ACTIONS[56]->object = &object_0056AF50;
	___SONIC_ACTIONS[57]->object = &object_0056AF50;
	___SONIC_ACTIONS[58]->object = &object_0056AF50;
	___SONIC_ACTIONS[59]->object = &object_0056AF50;
	___SONIC_ACTIONS[60]->object = &object_0056AF50;
	___SONIC_ACTIONS[61]->object = &object_0056AF50;
	___SONIC_ACTIONS[62]->object = &object_0056AF50;
	___SONIC_ACTIONS[63]->object = &object_0056AF50;
	___SONIC_ACTIONS[64]->object = &object_0056AF50;
	___SONIC_ACTIONS[65]->object = &object_0056AF50;
	___SONIC_ACTIONS[66]->object = &object_0056AF50;
	___SONIC_ACTIONS[67]->object = &object_0056AF50;
	___SONIC_ACTIONS[68]->object = &object_0056AF50;
	___SONIC_ACTIONS[69]->object = &object_0056AF50;
	___SONIC_ACTIONS[70]->object = &object_0056AF50;
	___SONIC_ACTIONS[71]->object = &object_0056AF50;
	___SONIC_ACTIONS[72]->object = &object_0056AF50;
	___SONIC_ACTIONS[87]->object = &object_0056AF50;
	___SONIC_ACTIONS[88]->object = &object_0056AF50;
	___SONIC_ACTIONS[89]->object = &object_0056AF50;
	___SONIC_ACTIONS[90]->object = &object_0056AF50;
	___SONIC_ACTIONS[91]->object = &object_0056AF50;
	___SONIC_ACTIONS[92]->object = &object_0056AF50;
	___SONIC_ACTIONS[93]->object = &object_0056AF50;
	___SONIC_ACTIONS[94]->object = &object_0056AF50;
	___SONIC_ACTIONS[95]->object = &object_0056AF50;
	___SONIC_ACTIONS[96]->object = &object_0056AF50;
	___SONIC_ACTIONS[97]->object = &object_0056AF50;
	___SONIC_ACTIONS[98]->object = &object_0056AF50;
	___SONIC_ACTIONS[99]->object = &object_0056AF50;
	___SONIC_ACTIONS[100]->object = &object_0056AF50;
	___SONIC_ACTIONS[101]->object = &object_0056AF50;
	___SONIC_ACTIONS[102]->object = &object_0056AF50;
	___SONIC_ACTIONS[103]->object = &object_0056AF50;
	___SONIC_ACTIONS[104]->object = &object_0056AF50;
	___SONIC_ACTIONS[105]->object = &object_0056AF50;
	___SONIC_ACTIONS[106]->object = &object_0056AF50;
	___SONIC_ACTIONS[107]->object = &object_0056AF50;
	___SONIC_ACTIONS[108]->object = &object_0056AF50;
	___SONIC_ACTIONS[109]->object = &object_0056AF50;
	___SONIC_ACTIONS[113]->object = &object_0056AF50;
	___SONIC_ACTIONS[114]->object = &object_0056AF50;
	___SONIC_ACTIONS[115]->object = &object_0056AF50;
	___SONIC_ACTIONS[116]->object = &object_0056AF50;
	___SONIC_ACTIONS[117]->object = &object_0056AF50;
	___SONIC_ACTIONS[118]->object = &object_0056AF50;
	___SONIC_ACTIONS[119]->object = &object_0056AF50;
	___SONIC_ACTIONS[120]->object = &object_0056AF50;
	___SONIC_ACTIONS[121]->object = &object_0056AF50;
	___SONIC_ACTIONS[122]->object = &object_0056AF50;
	___SONIC_ACTIONS[123]->object = &object_0056AF50;
	___SONIC_ACTIONS[124]->object = &object_0056AF50;
	___SONIC_ACTIONS[125]->object = &object_0056AF50;
	___SONIC_ACTIONS[126]->object = &object_0056AF50;
	___SONIC_ACTIONS[127]->object = &object_0056AF50;
	___SONIC_ACTIONS[128]->object = &object_0056AF50;
	___SONIC_ACTIONS[129]->object = &object_0056AF50;
	___SONIC_ACTIONS[134]->object = &object_0056AF50;
	___SONIC_ACTIONS[135]->object = &object_0056AF50;
	___SONIC_ACTIONS[136]->object = &object_0056AF50;
	___SONIC_ACTIONS[137]->object = &object_0056AF50;
	___SONIC_ACTIONS[145]->object = &object_0056AF50;
	___SONIC_ACTIONS[146]->object = &object_0056AF50;
	___SONIC_ACTIONS[147]->object = &object_0056AF50;
	___SONIC_ACTIONS[148]->object = &object_0056AF50;
	___SONIC_MODELS[0] = &attach_0055F304;
	___SONIC_MODELS[1] = &attach_00560DA4;
	___SONIC_MODELS[2] = &attach_005735AC;
	___SONIC_MODELS[3] = &attach_00573DFC;
	___SONIC_MODELS[4] = &attach_0057464C;
	___SONIC_MODELS[5] = &attach_0057525C;
	___SONIC_MODELS[6] = &attach_00575AB4;
	___SONIC_MODELS[7] = &attach_0057630C;
	___SONIC_MODELS[8] = &attach_00569568;
	___SONIC_MODELS[9] = &attach_00579C68;

	//SuperSonic
	___SONIC_OBJECTS[22] = &object_0062DE88;
	___SONIC_OBJECTS[23] = &object_00626AB4;
	___SONIC_OBJECTS[24] = &object_00626C44;
	___SONIC_OBJECTS[25] = &object_0062840C;
	___SONIC_OBJECTS[26] = &object_00627BF0;
	___SONIC_OBJECTS[27] = &object_006276D8;
	___SONIC_OBJECTS[28] = &object_00624E3C;
	___SONIC_OBJECTS[29] = &object_00624FCC;
	___SONIC_OBJECTS[30] = &object_006267F4;
	___SONIC_OBJECTS[31] = &object_00625FD8;
	___SONIC_OBJECTS[32] = &object_00625AA8;
	___SONIC_OBJECTS[33] = &object_00623474;
	___SONIC_OBJECTS[34] = &object_00623604;
	___SONIC_OBJECTS[35] = &object_00624B78;
	___SONIC_OBJECTS[36] = &object_00624308;
	___SONIC_OBJECTS[37] = &object_00623C14;
	___SONIC_OBJECTS[38] = &object_00621AC4;
	___SONIC_OBJECTS[39] = &object_00621C54;
	___SONIC_OBJECTS[40] = &object_006231E0;
	___SONIC_OBJECTS[41] = &object_00622970;
	___SONIC_OBJECTS[42] = &object_00622254;
	___SONIC_ACTIONS[130]->object = &object_0062DE88;
	___SONIC_ACTIONS[131]->object = &object_0062DE88;
	___SONIC_ACTIONS[132]->object = &object_0062DE88;
	___SONIC_ACTIONS[133]->object = &object_0062DE88;
	___SONIC_ACTIONS[138]->object = &object_0062DE88;
	___SONIC_ACTIONS[139]->object = &object_0062DE88;
	___SONIC_ACTIONS[140]->object = &object_0062DE88;
	___SONIC_ACTIONS[141]->object = &object_0062DE88;
	___SONIC_ACTIONS[143]->object = &object_0062DE88;
	___SONIC_ACTIONS[144]->object = &object_0062DE88;

	//MILES
	NJS_OBJECT **___MILES_OBJECTS = (NJS_OBJECT **)GetProcAddress(handle, "___MILES_OBJECTS");
	NJS_ACTION **___MILES_ACTIONS = (NJS_ACTION **)GetProcAddress(handle, "___MILES_ACTIONS");
	NJS_MODEL_SADX **___MILES_MODELS = (NJS_MODEL_SADX **)GetProcAddress(handle, "___MILES_MODELS");
	___MILES_OBJECTS[0] = &object_0042AD54;
	___MILES_OBJECTS[1] = &object_00437C44;
	___MILES_OBJECTS[2] = &object_0043F4B4;
	___MILES_OBJECTS[3] = &object_0044148C;
	___MILES_OBJECTS[4] = &object_0042ABE8;
	___MILES_OBJECTS[5] = &object_00437AD8;
	___MILES_OBJECTS[6] = &object_0042AC84;
	___MILES_OBJECTS[7] = &object_00437B74;
	___MILES_OBJECTS[8] = &object_00426ED0;
	___MILES_OBJECTS[9] = &object_00426B90;
	___MILES_OBJECTS[10] = &object_00433DC0;
	___MILES_OBJECTS[11] = &object_00433A80;
	___MILES_OBJECTS[12] = &object_0041E548;
	___MILES_OBJECTS[13] = &object_0041EC90;
	___MILES_OBJECTS[14] = &object_0041ED2C;
	___MILES_OBJECTS[15] = &object_0041E5E4;
	___MILES_OBJECTS[16] = &object_0042BC1C;
	___MILES_OBJECTS[17] = &object_0042B4D4;
	___MILES_OBJECTS[18] = &object_0042BBE8;
	___MILES_OBJECTS[19] = &object_0042B4A0;
	___MILES_OBJECTS[20] = &object_00424D00;
	___MILES_OBJECTS[21] = &object_00424E94;
	___MILES_OBJECTS[22] = &object_0042674C;
	___MILES_OBJECTS[23] = &object_00422FD8;
	___MILES_OBJECTS[24] = &object_0042316C;
	___MILES_OBJECTS[25] = &object_00424A14;
	___MILES_OBJECTS[26] = &object_0042182C;
	___MILES_OBJECTS[27] = &object_004219BC;
	___MILES_OBJECTS[28] = &object_00422CF0;
	___MILES_OBJECTS[29] = &object_004200CC;
	___MILES_OBJECTS[30] = &object_0042025C;
	___MILES_OBJECTS[31] = &object_00421598;
	___MILES_OBJECTS[32] = &object_004225C8;
	___MILES_OBJECTS[33] = &object_00421FE4;
	___MILES_OBJECTS[34] = &object_00420E70;
	___MILES_OBJECTS[35] = &object_00420884;
	___MILES_OBJECTS[36] = &object_00425E88;
	___MILES_OBJECTS[37] = &object_00425C40;
	___MILES_OBJECTS[38] = &object_00424158;
	___MILES_OBJECTS[39] = &object_00423F10;
	___MILES_OBJECTS[40] = &object_00431BF0;
	___MILES_OBJECTS[41] = &object_00431D84;
	___MILES_OBJECTS[42] = &object_0043363C;
	___MILES_OBJECTS[43] = &object_0042FEC8;
	___MILES_OBJECTS[44] = &object_0043005C;
	___MILES_OBJECTS[45] = &object_00431904;
	___MILES_OBJECTS[46] = &object_0042E71C;
	___MILES_OBJECTS[47] = &object_0042E8AC;
	___MILES_OBJECTS[48] = &object_0042FBE0;
	___MILES_OBJECTS[49] = &object_0042CFBC;
	___MILES_OBJECTS[50] = &object_0042D14C;
	___MILES_OBJECTS[51] = &object_0042E488;
	___MILES_OBJECTS[52] = &object_0042F4B8;
	___MILES_OBJECTS[53] = &object_0042EED4;
	___MILES_OBJECTS[54] = &object_0042DD60;
	___MILES_OBJECTS[55] = &object_0042D774;
	___MILES_OBJECTS[56] = &object_00432D78;
	___MILES_OBJECTS[57] = &object_00432B30;
	___MILES_OBJECTS[58] = &object_00431048;
	___MILES_OBJECTS[59] = &object_00430E00;
	___MILES_OBJECTS[60] = &object_0046E63C;
	___MILES_OBJECTS[61] = &object_0046F46C;
	___MILES_OBJECTS[62] = &object_0041FE04;
	___MILES_OBJECTS[63] = &object_0042CCF4;
	___MILES_OBJECTS[64] = &object_0046EE44;
	___MILES_OBJECTS[65] = &object_0046FC84;
	___MILES_OBJECTS[66] = &object_0042AC1C;
	___MILES_OBJECTS[67] = &object_0042ABE8;
	___MILES_OBJECTS[68] = &object_00437B0C;
	___MILES_OBJECTS[69] = &object_00437AD8;
	___MILES_OBJECTS[70] = &object_0044C724;
	___MILES_OBJECTS[71] = &object_0055C92C;
	___MILES_ACTIONS[0]->object = &object_00437C44;
	___MILES_ACTIONS[1]->object = &object_00437C44;
	___MILES_ACTIONS[2]->object = &object_00437C44;
	___MILES_ACTIONS[3]->object = &object_00437C44;
	___MILES_ACTIONS[4]->object = &object_00437C44;
	___MILES_ACTIONS[5]->object = &object_00437C44;
	___MILES_ACTIONS[6]->object = &object_00437C44;
	___MILES_ACTIONS[7]->object = &object_00437C44;
	___MILES_ACTIONS[8]->object = &object_00437C44;
	___MILES_ACTIONS[9]->object = &object_00437C44;
	___MILES_ACTIONS[10]->object = &object_00437C44;
	___MILES_ACTIONS[11]->object = &object_0042AD54;
	___MILES_ACTIONS[12]->object = &object_0042AD54;
	___MILES_ACTIONS[13]->object = &object_0042AD54;
	___MILES_ACTIONS[14]->object = &object_0042AD54;
	___MILES_ACTIONS[15]->object = &object_0042AD54;
	___MILES_ACTIONS[16]->object = &object_0042AD54;
	___MILES_ACTIONS[17]->object = &object_00437C44;
	___MILES_ACTIONS[18]->object = &object_00437C44;
	___MILES_ACTIONS[19]->object = &object_00437C44;
	___MILES_ACTIONS[20]->object = &object_00437C44;
	___MILES_ACTIONS[21]->object = &object_0042AD54;
	___MILES_ACTIONS[22]->object = &object_0042AD54;
	___MILES_ACTIONS[23]->object = &object_0042AD54;
	___MILES_ACTIONS[24]->object = &object_0042AD54;
	___MILES_ACTIONS[25]->object = &object_0042AD54;
	___MILES_ACTIONS[26]->object = &object_0042AD54;
	___MILES_ACTIONS[27]->object = &object_0042AD54;
	___MILES_ACTIONS[28]->object = &object_0042AD54;
	___MILES_ACTIONS[29]->object = &object_0042AD54;
	___MILES_ACTIONS[30]->object = &object_0042AD54;
	___MILES_ACTIONS[31]->object = &object_0042AD54;
	___MILES_ACTIONS[32]->object = &object_0042AD54;
	___MILES_ACTIONS[33]->object = &object_0042AD54;
	___MILES_ACTIONS[34]->object = &object_0042AD54;
	___MILES_ACTIONS[35]->object = &object_0042AD54;
	___MILES_ACTIONS[36]->object = &object_0042AD54;
	___MILES_ACTIONS[37]->object = &object_0042AD54;
	___MILES_ACTIONS[38]->object = &object_0042AD54;
	___MILES_ACTIONS[39]->object = &object_0042AD54;
	___MILES_ACTIONS[40]->object = &object_0042AD54;
	___MILES_ACTIONS[41]->object = &object_0042AD54;
	___MILES_ACTIONS[42]->object = &object_0042AD54;
	___MILES_ACTIONS[43]->object = &object_0042AD54;
	___MILES_ACTIONS[44]->object = &object_0042AD54;
	___MILES_ACTIONS[45]->object = &object_0042AD54;
	___MILES_ACTIONS[46]->object = &object_0042AD54;
	___MILES_ACTIONS[47]->object = &object_0042AD54;
	___MILES_ACTIONS[48]->object = &object_0042AD54;
	___MILES_ACTIONS[49]->object = &object_0042AD54;
	___MILES_ACTIONS[50]->object = &object_0042AD54;
	___MILES_ACTIONS[51]->object = &object_0042AD54;
	___MILES_ACTIONS[52]->object = &object_0042AD54;
	___MILES_ACTIONS[53]->object = &object_00437C44;
	___MILES_ACTIONS[54]->object = &object_00437C44;
	___MILES_ACTIONS[55]->object = &object_0042AD54;
	___MILES_ACTIONS[56]->object = &object_0042AD54;
	___MILES_ACTIONS[57]->object = &object_0042AD54;
	___MILES_ACTIONS[58]->object = &object_0042AD54;
	___MILES_ACTIONS[59]->object = &object_0042AD54;
	___MILES_ACTIONS[60]->object = &object_0042AD54;
	___MILES_ACTIONS[61]->object = &object_0042AD54;
	___MILES_ACTIONS[62]->object = &object_0042AD54;
	___MILES_ACTIONS[63]->object = &object_0042AD54;
	___MILES_ACTIONS[64]->object = &object_0043F4B4;
	___MILES_ACTIONS[65]->object = &object_0044148C;
	___MILES_ACTIONS[66]->object = &object_0042AD54;
	___MILES_ACTIONS[67]->object = &object_0042AD54;
	___MILES_ACTIONS[68]->object = &object_0042AD54;
	___MILES_ACTIONS[69]->object = &object_0042AD54;
	___MILES_ACTIONS[70]->object = &object_0042AD54;
	___MILES_ACTIONS[71]->object = &object_0042AD54;
	___MILES_ACTIONS[72]->object = &object_0042AD54;
	___MILES_ACTIONS[73]->object = &object_0042AD54;
	___MILES_ACTIONS[74]->object = &object_0042AD54;
	___MILES_ACTIONS[75]->object = &object_0042AD54;
	___MILES_ACTIONS[76]->object = &object_0042AD54;
	___MILES_ACTIONS[77]->object = &object_0042AD54;
	___MILES_ACTIONS[78]->object = &object_0042AD54;
	___MILES_ACTIONS[79]->object = &object_0042AD54;
	___MILES_ACTIONS[80]->object = &object_0042AD54;
	___MILES_ACTIONS[81]->object = &object_0042AD54;
	___MILES_ACTIONS[82]->object = &object_0042AD54;
	___MILES_ACTIONS[83]->object = &object_0042AD54;
	___MILES_ACTIONS[85]->object = &object_0042AD54;
	___MILES_ACTIONS[86]->object = &object_0042AD54;
	___MILES_ACTIONS[87]->object = &object_0042AD54;
	___MILES_ACTIONS[88]->object = &object_0042AD54;
	___MILES_ACTIONS[91]->object = &object_0042AD54;
	___MILES_ACTIONS[92]->object = &object_0042AD54;
	___MILES_ACTIONS[93]->object = &object_0042AD54;
	___MILES_ACTIONS[94]->object = &object_0042AD54;
	___MILES_ACTIONS[95]->object = &object_0042AD54;
	___MILES_ACTIONS[96]->object = &object_0042AD54;
	___MILES_ACTIONS[97]->object = &object_0042AD54;
	___MILES_ACTIONS[100]->object = &object_0055C92C;
	___MILES_ACTIONS[101]->object = &object_0055C92C;
	___MILES_ACTIONS[102]->object = &object_0055C92C;
	___MILES_ACTIONS[103]->object = &object_0055C92C;
	___MILES_ACTIONS[104]->object = &object_0055C92C;
	___MILES_ACTIONS[105]->object = &object_0055C92C;
	___MILES_ACTIONS[106]->object = &object_0055C92C;
	___MILES_ACTIONS[107]->object = &object_0055C92C;
	___MILES_ACTIONS[108]->object = &object_0055C92C;
	___MILES_ACTIONS[109]->object = &object_0055C92C;
	___MILES_ACTIONS[110]->object = &object_0042AD54;
	___MILES_ACTIONS[111]->object = &object_0042AD54;
	___MILES_ACTIONS[112]->object = &object_0042AD54;
	___MILES_ACTIONS[113]->object = &object_0042AD54;
	___MILES_MODELS[0] = &attach_0041FDD8;
	___MILES_MODELS[1] = &attach_0042CCC8;
	___MILES_MODELS[2] = &attach_0042ABBC;
	___MILES_MODELS[3] = &attach_00437AAC;
	___MILES_MODELS[4] = &attach_0044517C;
	___MILES_MODELS[5] = &attach_0041E51C;
	___MILES_MODELS[6] = &attach_0041EC64;
	___MILES_MODELS[7] = &attach_0042B40C;
	___MILES_MODELS[8] = &attach_0042BB54;
	___MILES_MODELS[9] = &attach_00445778;
	___MILES_MODELS[10] = &attach_00445DA8;
	___MILES_MODELS[11] = &attach_00446A08;
	___MILES_MODELS[12] = &attach_00447038;
	___MILES_MODELS[13] = &attach_004463D8;
	___MILES_MODELS[14] = &attach_0046DFE8;

	//Knuckles
	NJS_OBJECT **___KNUCKLES_OBJECTS = (NJS_OBJECT **)GetProcAddress(handle, "___KNUCKLES_OBJECTS");
	NJS_ACTION **___KNUCKLES_ACTIONS = (NJS_ACTION **)GetProcAddress(handle, "___KNUCKLES_ACTIONS");
	NJS_MODEL_SADX **___KNUCKLES_MODELS = (NJS_MODEL_SADX **)GetProcAddress(handle, "___KNUCKLES_MODELS");
	___KNUCKLES_OBJECTS[0] = &object_002E23B0;
	___KNUCKLES_OBJECTS[1] = &object_002EEE50;
	___KNUCKLES_OBJECTS[2] = &object_002DB8A4;
	___KNUCKLES_OBJECTS[3] = &object_002DBA34;
	___KNUCKLES_OBJECTS[4] = &object_002DD0AC;
	___KNUCKLES_OBJECTS[5] = &object_002DC244;
	___KNUCKLES_OBJECTS[6] = &object_002D9E5C;
	___KNUCKLES_OBJECTS[7] = &object_002D9FEC;
	___KNUCKLES_OBJECTS[8] = &object_002DB5F4;
	___KNUCKLES_OBJECTS[9] = &object_002DAB34;
	___KNUCKLES_OBJECTS[10] = &object_002D876C;
	___KNUCKLES_OBJECTS[11] = &object_002D88FC;
	___KNUCKLES_OBJECTS[12] = &object_002D9BB0;
	___KNUCKLES_OBJECTS[13] = &object_002D6FE4;
	___KNUCKLES_OBJECTS[14] = &object_002D7174;
	___KNUCKLES_OBJECTS[15] = &object_002D8438;
	___KNUCKLES_OBJECTS[16] = &object_002D9754;
	___KNUCKLES_OBJECTS[17] = &object_002D9088;
	___KNUCKLES_OBJECTS[18] = &object_002D7FDC;
	___KNUCKLES_OBJECTS[19] = &object_002D7900;
	___KNUCKLES_OBJECTS[20] = &object_002DC94C;
	___KNUCKLES_OBJECTS[21] = &object_002DC244;
	___KNUCKLES_OBJECTS[22] = &object_002DAEE4;
	___KNUCKLES_OBJECTS[23] = &object_002DAB34;
	___KNUCKLES_OBJECTS[24] = &object_002E8324;
	___KNUCKLES_OBJECTS[25] = &object_002E84B4;
	___KNUCKLES_OBJECTS[26] = &object_002E9B2C;
	___KNUCKLES_OBJECTS[27] = &object_002E8CC4;
	___KNUCKLES_OBJECTS[28] = &object_002E68DC;
	___KNUCKLES_OBJECTS[29] = &object_002E6A6C;
	___KNUCKLES_OBJECTS[30] = &object_002E8074;
	___KNUCKLES_OBJECTS[31] = &object_002E75B4;
	___KNUCKLES_OBJECTS[32] = &object_002E51EC;
	___KNUCKLES_OBJECTS[33] = &object_002E537C;
	___KNUCKLES_OBJECTS[34] = &object_002E6630;
	___KNUCKLES_OBJECTS[35] = &object_002E3A64;
	___KNUCKLES_OBJECTS[36] = &object_002E3BF4;
	___KNUCKLES_OBJECTS[37] = &object_002E4EB8;
	___KNUCKLES_OBJECTS[38] = &object_002E61D4;
	___KNUCKLES_OBJECTS[39] = &object_002E5B08;
	___KNUCKLES_OBJECTS[40] = &object_002E4A5C;
	___KNUCKLES_OBJECTS[41] = &object_002E4380;
	___KNUCKLES_OBJECTS[42] = &object_002E93CC;
	___KNUCKLES_OBJECTS[43] = &object_002E8CC4;
	___KNUCKLES_OBJECTS[44] = &object_002E7964;
	___KNUCKLES_OBJECTS[45] = &object_002E75B4;
	___KNUCKLES_OBJECTS[46] = &object_00328878;
	___KNUCKLES_OBJECTS[47] = &object_003291C4;
	___KNUCKLES_OBJECTS[49] = &object_002E191C;
	___KNUCKLES_OBJECTS[50] = &object_002EE3BC;
	___KNUCKLES_OBJECTS[51] = &object_002E21A8;
	___KNUCKLES_OBJECTS[52] = &object_002E2070;
	___KNUCKLES_OBJECTS[53] = &object_002E1C78;
	___KNUCKLES_OBJECTS[54] = &object_002E1D14;
	___KNUCKLES_OBJECTS[55] = &object_002E191C;
	___KNUCKLES_OBJECTS[56] = &object_0030257C;
	___KNUCKLES_OBJECTS[58] = &object_002DB5F4;
	___KNUCKLES_OBJECTS[59] = &object_002DAEE4;
	___KNUCKLES_OBJECTS[60] = &object_002DAB34;
	___KNUCKLES_OBJECTS[61] = &object_002DA324;
	___KNUCKLES_OBJECTS[62] = &object_002DD0AC;
	___KNUCKLES_OBJECTS[63] = &object_002DC94C;
	___KNUCKLES_OBJECTS[64] = &object_002DC244;
	___KNUCKLES_OBJECTS[65] = &object_002DC59C;
	___KNUCKLES_OBJECTS[66] = &object_002E8074;
	___KNUCKLES_OBJECTS[67] = &object_002E7964;
	___KNUCKLES_OBJECTS[68] = &object_002E75B4;
	___KNUCKLES_OBJECTS[69] = &object_002E6DA4;
	___KNUCKLES_OBJECTS[70] = &object_002E9B2C;
	___KNUCKLES_OBJECTS[71] = &object_002E93CC;
	___KNUCKLES_OBJECTS[72] = &object_002E8CC4;
	___KNUCKLES_OBJECTS[73] = &object_002E901C;
	___KNUCKLES_ACTIONS[0]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[1]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[2]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[3]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[4]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[5]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[6]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[7]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[9]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[10]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[11]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[12]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[13]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[14]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[15]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[16]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[17]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[18]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[19]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[20]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[21]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[22]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[23]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[24]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[25]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[26]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[27]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[28]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[29]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[30]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[31]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[32]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[33]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[34]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[35]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[36]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[37]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[38]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[39]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[40]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[41]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[42]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[43]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[44]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[45]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[46]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[47]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[48]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[49]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[50]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[51]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[52]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[53]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[54]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[55]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[56]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[57]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[58]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[59]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[60]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[61]->object = &object_002F8530;
	___KNUCKLES_ACTIONS[62]->object = &object_002F0E24;
	___KNUCKLES_ACTIONS[63]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[64]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[65]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[66]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[67]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[68]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[69]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[70]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[71]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[72]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[73]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[74]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[75]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[76]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[77]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[78]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[79]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[80]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[81]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[82]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[83]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[84]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[85]->object = &object_002EEE50;
	___KNUCKLES_ACTIONS[86]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[87]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[88]->object = &object_002E23B0;
	___KNUCKLES_ACTIONS[89]->object = &object_002E23B0;
	___KNUCKLES_MODELS[0] = &attach_003298C8;
	___KNUCKLES_MODELS[1] = &attach_00329FF8;
	___KNUCKLES_MODELS[2] = &attach_0032A6F0;
	___KNUCKLES_MODELS[3] = &attach_0032ADF8;
	___KNUCKLES_MODELS[4] = &attach_002E18F0;
	___KNUCKLES_MODELS[5] = &attach_002EE390;
	___KNUCKLES_MODELS[6] = &attach_002FC558;
	___KNUCKLES_MODELS[7] = &attach_002DB5C8;
	___KNUCKLES_MODELS[8] = &attach_002DAEB8;
	___KNUCKLES_MODELS[9] = &attach_002DAB08;
	___KNUCKLES_MODELS[10] = &attach_002DA2F8;
	___KNUCKLES_MODELS[11] = &attach_002DD080;
	___KNUCKLES_MODELS[12] = &attach_002DC920;
	___KNUCKLES_MODELS[13] = &attach_002DC218;
	___KNUCKLES_MODELS[14] = &attach_002DC570;
	___KNUCKLES_MODELS[15] = &attach_0032B488;
	___KNUCKLES_MODELS[16] = &attach_0032C2F8;
	___KNUCKLES_MODELS[17] = &attach_0032BF48;
	___KNUCKLES_MODELS[18] = &attach_0032B7C0;
	___KNUCKLES_MODELS[19] = &attach_0032C950;
	___KNUCKLES_MODELS[20] = &attach_0032D7E0;
	___KNUCKLES_MODELS[21] = &attach_0032D430;
	___KNUCKLES_MODELS[22] = &attach_0032CCA8;

	ReplacePVM("SONIC", "SONIC_Classic");
	ReplacePVM("SUPERSONIC", "SUPERSONIC_Classic");
	ReplacePVM("SUPERSONIC_DC", "SUPERSONIC_Classic");
	ReplacePVM("MILES_DC", "MILES_Classic");
	ReplacePVM("MILES", "MILES_Classic");
	ReplacePVM("Knuckles", "Knuckles_Classic");
	ReplacePVM("Knuckles_DC", "Knuckles_Classic");
	//ReplacePVM("SONIC_DC", "SONIC_Classic");
	//ReplacePVM("SONIC_HQ", "SONIC_Classic");
	WriteJump((void*)0x007D0B50, InitSonicWeldInfo_mod);
	WriteJump((void*)0x00493500, Sonic_MorphStretchyFeet_asm);
	WriteJump((void*)0x0045B840, Tails_Jiggle_mod);
	WriteJump((void*)0x007C6D80, InitTailsWeldInfo_mod);
	//WriteJump((void*)0x007C94D0, InitKnucklesWeldInfo_mod);
	//WriteData((uint16_t**)0x0472CAC, (uint16_t*)&Knuckles_ShovelClawIndices_DC);
	//WriteData((uint16_t**)0x0472B2C, (uint16_t*)&Knuckles_HandIndices_DC);
	//WriteData((uint16_t**)0x0472835, (uint16_t*)&Knuckles_HandIndices_DC);
	//WriteData<1>((char*)0x007C6D80, 0xC3u); //Tails Welds Disable
	WriteData<1>((char*)0x007C94D0, 0xC3u); //Knuckles Welds Disable
	//WriteData<1>((char*)0x0045B840, 0xC3u);
	//WriteData<5>((char*)0x00461356, 0x90u);
	//WriteData<5>((char*)0x00461373, 0x90u);
}

extern "C" __declspec(dllexport) const ModInfo SADXModInfo = { ModLoaderVer };