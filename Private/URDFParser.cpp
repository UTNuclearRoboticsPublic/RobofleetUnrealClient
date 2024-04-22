#include "URDFParser.h"


UURDFParser::UURDFParser() {}

/* Takes raw URDF string and prepares it for parsing */
bool UURDFParser::loadURDF(const FString& urdf) {
	cleanXML(urdf);
	bool load_result = loadXML();
	return load_result;
}

/* Helper function to convert pugi::char_t* to string */
std::string UURDFParser::as_utf8(const char* str) { return str; }

/* Removes unnecessary spaces and characters from URDF string */
void UURDFParser::cleanXML(const FString& urdf) {

	std::string str(TCHAR_TO_ANSI(*urdf));

	std::string remA = "\\n";
	std::string remB = "\\";

	// Remove all instances of \n
	size_t pos = 0;
	while (pos += 1)
	{
		pos = str.find(remA, pos);
		if (pos == std::string::npos) { break; }
		str.erase(pos, remA.length());
	}

	// Remove all instances of \

	pos = 0;
	while (pos += 1)
	{
		pos = str.find(remB, pos);
		if (pos == std::string::npos) { break; }
		str.erase(pos, remB.length());
	}

	// Condense multiple whitespaces into one whitespace
	std::regex reg1(R"(\s+)");
	std::string s1 = std::regex_replace(str, reg1, " ");

	// Remove whitespaces between =" and whatever comes after
	std::regex reg2(R"(="\s+)");
	std::string s2 = std::regex_replace(s1, reg2, "=\"");

	cleaned_xml.assign(s2);
}

/* Use pugixml to parse cleaned xml string and build tree of xml_nodes */
bool UURDFParser::loadXML() {

	result = doc.load_string(cleaned_xml.c_str(), pugi::parse_default | pugi::parse_comments);

	if (!result) {
		UE_LOG(LogTemp, Warning, TEXT("Failed to parse XML string"));
		return false;
	}

	robot = doc.child("robot");

	if (!robot) {
		UE_LOG(LogTemp, Warning, TEXT("No <robot> tag found"));
		return false;
	}

	return true;
}

/* Returns list of ONLY the joints that are direct children of the robot tag */
TArray<FString> UURDFParser::getJointList() {

	TArray<FString> joint_list;

	if (robot) {
		for (pugi::xml_node joint = robot.child("joint"); joint; joint = joint.next_sibling("joint"))
		{
			FString vfs = as_utf8(joint.attribute("name").value()).c_str();
			joint_list.Add(vfs);
		}
	}

	return joint_list;
}

/* Returns list of ONLY the links that are direct children of the robot tag */
TArray<FString> UURDFParser::getLinkList() {

	if (robot) {
		for (pugi::xml_node link = robot.child("link"); link; link = link.next_sibling("link"))
		{
			FString vfs = as_utf8(link.attribute("name").value()).c_str();
			link_list.Add(vfs);
			//UE_LOG(LogTemp, Warning, TEXT("LINK LIST: %s"), *vfs);
		}
	}

	linksLoaded = true;

	return link_list;
}

/* Populates jointParent with the parent link of the specified joint.
	Returns false if specified joint does not exist. */
bool UURDFParser::getJointParent(const FString& jointName, FString& jointParent) {

	pugi::xml_node givenJoint = robot.find_child_by_attribute("joint", "name", TCHAR_TO_ANSI(*jointName));

	if (!givenJoint) { return false; }

	jointParent = as_utf8(givenJoint.child("parent").attribute("link").value()).c_str();
	UE_LOG(LogTemp, Warning, TEXT("JOINT PARENT: %s"), *jointParent);

	return true;
}

/* Populates position and orientation vectors with the components of the transform of the specified joint
	Returns false if transform tag doesn't exist. */
bool UURDFParser::getJointTransform(const FString& jointName, FVector& position, FVector& orientation) {

	pugi::xml_node givenJoint = robot.find_child_by_attribute("joint", "name", TCHAR_TO_ANSI(*jointName));

	const pugi::char_t* rpy_node = givenJoint.child("origin").attribute("rpy").value();
	const pugi::char_t* xyz_node = givenJoint.child("origin").attribute("xyz").value();

	std::string rpy = as_utf8(rpy_node);
	std::string xyz = as_utf8(xyz_node);

	const char* delim = " ";
	const char* token = strtok(const_cast<char*>(rpy.c_str()), delim);

	if (!token) { return false; }

	orientation.X = atof(token);

	token = strtok(nullptr, delim);
	orientation.Y = atof(token);

	token = strtok(nullptr, delim);
	orientation.Z = atof(token);

	// now do xyz
	token = strtok(const_cast<char*>(xyz.c_str()), delim);

	if (!token) { return false; }

	position.X = atof(token);

	token = strtok(nullptr, delim);
	position.Y = atof(token);

	token = strtok(nullptr, delim);
	position.Z = atof(token);

	UE_LOG(LogTemp, Warning, TEXT("JOINT RPY: %f %f %f"), orientation.X, orientation.Y, orientation.Z);
	UE_LOG(LogTemp, Warning, TEXT("JOINT XYZ: %f %f %f"), position.X, position.Y, position.Z);

	return true;
}

bool UURDFParser::getBaseLink(FString& baselink) {

	// check if link list loaded
	if (!linksLoaded) {
		getLinkList();
	}

	// check if base link already found
	if (baseLoaded) {
		baselink = base_link;
		UE_LOG(LogTemp, Warning, TEXT("BASE LINK FOUND: %s"), *base_link);
		return true;
	}

	// go through link names (not the actual xml nodes!)
	for (auto& link : link_list) {
		bool is_child = false;
		bool is_parent = false;
		char* attr_value = TCHAR_TO_ANSI(*link);

		// go through all joint xml nodes looking for that link name
		for (pugi::xml_node joint = robot.child("joint"); joint; joint = joint.next_sibling("joint"))
		{
			// check if this link shows up as a child, if so, then break (bc base link can't ever be a child)
			is_child = joint.find_child_by_attribute("child", "link", attr_value) != NULL;

			if (is_child) {
				break;
			}

			// if not child, check link is a parent at least once (to confirm base link is used somewhere)
			if (!is_parent) {
				is_parent = joint.find_child_by_attribute("parent", "link", attr_value) != NULL;
			}

		}

		// if link is not child anywhere and is a parent somewhere, it is the base link
		if (!is_child && is_parent) {
			base_link = link;
			baseLoaded = true;
			UE_LOG(LogTemp, Warning, TEXT("BASE LINK FOUND: %s"), *link);
			baselink = base_link;
			return true;
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("NO BASE LINK FOUND"));

	return false;

}