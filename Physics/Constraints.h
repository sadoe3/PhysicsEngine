//
//	Constraints.h
//
#pragma once
#include "Constraints/ConstraintConstantVelocity.h"
#include "Constraints/ConstraintDistance.h"
#include "Constraints/ConstraintHinge.h"
#include "Constraints/ConstraintSpinner.h"
#include "Constraints/ConstraintMover.h"
#include "Constraints/ConstraintOrientation.h"
#include "Constraints/ConstraintPenetration.h"

int CompareContacts(const void* contactA, const void* contactB);