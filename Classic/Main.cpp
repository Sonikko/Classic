#include "stdafx.h"
#include "include/SADXModLoader.h"
#include "Sonic.h"
#include "SuperSonic.h"
#include "Functions.h"

#define ReplacePVM(a)  helperFunctions.ReplaceFile("system\\" a ".PVM", "system\\" a "_Classic.PVM");


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
	ReplacePVM("SONIC", "SONIC_Classic");
	ReplacePVM("SUPERSONIC", "SUPERSONIC_Classic");
	ReplacePVM("SUPERSONIC_DC", "SUPERSONIC_Classic");
	//ReplacePVM("SONIC_DC", "SONIC_Classic");
	//ReplacePVM("SONIC_HQ", "SONIC_Classic");
	WriteJump((void*)0x007D0B50, InitSonicWeldInfo_mod);
	WriteJump((void*)0x00493500, Sonic_MorphStretchyFeet_asm);
}

extern "C" __declspec(dllexport) const ModInfo SADXModInfo = { ModLoaderVer };