#include "slalom.h"

using namespace ctrl;

static auto SS_SL90 = slalom::Shape(Position(45, 45, M_PI / 2), 44);
static auto SS_SR90 = slalom::Shape(Position(45, -45, -M_PI / 2), -44);
static auto SS_FL45 = slalom::Shape(Position(90, 45, M_PI / 4), 31);
static auto SS_FR45 = slalom::Shape(Position(90, -45, -M_PI / 4), -31);
static auto SS_FL90 = slalom::Shape(Position(90, 90, M_PI / 2), 70);
static auto SS_FR90 = slalom::Shape(Position(90, -90, -M_PI / 2), -70);
static auto SS_FL135 = slalom::Shape(Position(45, 90, M_PI * 3 / 4), 80);
static auto SS_FR135 = slalom::Shape(Position(45, -90, -M_PI * 3 / 4), -80);
static auto SS_FL180 = slalom::Shape(Position(0, 90, M_PI), 90, 24);
static auto SS_FR180 = slalom::Shape(Position(0, -90, -M_PI), -90, 24);
static auto SS_FLV90 =
    slalom::Shape(Position(45 * std::sqrt(2), 45 * std::sqrt(2), M_PI / 2), 50);
static auto SS_FRV90 = slalom::Shape(
    Position(45 * std::sqrt(2), -45 * std::sqrt(2), -M_PI / 2), -50);
static auto SS_FLS90 = slalom::Shape(Position(45, 45, M_PI / 2), 44.5);
static auto SS_FRS90 = slalom::Shape(Position(45, -45, -M_PI / 2), -44.5);

#define TO_STRING(VariableName) #VariableName

int main(void) {
  SS_SL90.printDefinition(std::cout, TO_STRING(SS_SL90));
  SS_SR90.printDefinition(std::cout, TO_STRING(SS_SR90));
  SS_FL45.printDefinition(std::cout, TO_STRING(SS_FL45));
  SS_FR45.printDefinition(std::cout, TO_STRING(SS_FR45));
  SS_FL90.printDefinition(std::cout, TO_STRING(SS_FL90));
  SS_FR90.printDefinition(std::cout, TO_STRING(SS_FR90));
  SS_FL135.printDefinition(std::cout, TO_STRING(SS_FL135));
  SS_FR135.printDefinition(std::cout, TO_STRING(SS_FR135));
  SS_FL180.printDefinition(std::cout, TO_STRING(SS_FL180));
  SS_FR180.printDefinition(std::cout, TO_STRING(SS_FR180));
  SS_FLV90.printDefinition(std::cout, TO_STRING(SS_FLV90));
  SS_FRV90.printDefinition(std::cout, TO_STRING(SS_FRV90));
  SS_FLS90.printDefinition(std::cout, TO_STRING(SS_FLS90));
  SS_FRS90.printDefinition(std::cout, TO_STRING(SS_FRS90));
  return 0;
}
