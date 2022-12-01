#pragma once

#include "pugixml_lib/include/pugixml.hpp"
#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"

#include <string>
#include <iostream>
#include <regex>
#include "URDFParser.generated.h"


UCLASS(BlueprintType)
class UURDFParser : public UObject
{
	GENERATED_BODY()

public:
	UURDFParser ();
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (ExposeOnSpawn = "true"))
	FString RepresentationID;

	UFUNCTION(BlueprintCallable, Category = "Parser")
	bool loadURDF(const FString &urdf);

	UFUNCTION(BlueprintCallable, Category = "Parser")
	TArray<FString> getJointList();

	UFUNCTION(BlueprintCallable, Category = "Parser")
	TArray<FString> getLinkList();

	UFUNCTION(BlueprintCallable, Category = "Parser")
	bool getJointParent(const FString &jointName, FString &jointParent);

	UFUNCTION(BlueprintCallable, Category = "Parser")
	bool getJointTransform(const FString &jointName, FVector &position, FVector &orientation);

private:

	pugi::xml_document doc;
	pugi::xml_parse_result result;
	std::string cleaned_xml;
	pugi::xml_node robot;

	std::string as_utf8(const char* str);
	bool loadXML();
	void cleanXML(const FString &urdf);

};