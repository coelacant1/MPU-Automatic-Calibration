#include "stdafx.h"
#include "CppUnitTest.h"
#include "../GimbalProcessor/GimbalManager.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace GimbalProcessorTest
{		
	TEST_CLASS(UnitTest1)
	{
	public:
		
		TEST_METHOD(GimbalAccelerometerTest)
		{
			GimbalManager gm = GimbalManager(false);

			gm.SetStartEndPosition(Vector3D(0, 0, 0), Vector3D(0, 1000, 0));
			gm.SetStartEndRotation(Vector3D(0, 0, 0), Vector3D(360, 360, 360), EulerConstants::EulerOrderXYZR);
			gm.SetTime(2500);

			for (double i = 0.0; i < 1.0; i += 0.01) {
				Logger::WriteMessage(gm.GetMPUAngularVelocity(i, 0.025).ToString().c_str());
			}
			// TODO: Your test code here
		}

	};
}