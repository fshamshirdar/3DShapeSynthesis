#include "control_points.h"

void ControlPointsMiner::scaleToTarget(Data::Part* ref, Data::Part* target)
{
	// Scale to target
	refBase = ref->boundingBox;
	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);

	refBaseScaled = ref->boundingBox;
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	Eigen::Vector3f translation = (target->boundingBox.min() - ref->boundingBox.min());
	ref->translate(translation);
	// end scaling
}

void ControlPointsMiner::unscaleToRef(Data::Part* ref, Data::Part* target)
{
	// Scale back to ref
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	Eigen::Vector3f translation = (refBaseScaled.min() - target->boundingBox.min());
	ref->translate(translation);

	Eigen::Vector3f baseScale = (refBase.min() + refBase.max()) / 2.;
	ref->scale(refBase, baseScale);
	// end un-scaling
}
