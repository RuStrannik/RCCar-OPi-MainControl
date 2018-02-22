import qbs

Project {
	minimumQbsVersion: "1.7.1"

	CppApplication {
		consoleApplication: true
		files: [
			"*.cpp",
//			"mcp.cpp",
//			"sunxi_gpio.cpp",
//			"softPwm.cpp",
//			"softServo.cpp",
		]

		Group {     // Properties for the produced executable
			fileTagsFilter: product.type
			qbs.install: true
		}
	}
}
