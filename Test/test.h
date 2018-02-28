#pragma once
#ifdef TEST_EXPORTS
#define KINECTVGB_API	__declspec(dllexport)
#else
#define KINECTVGB_API	__declspec(dllimport)
#endif // KINECTVGBEXPORT

KINECTVGB_API int GetValue();