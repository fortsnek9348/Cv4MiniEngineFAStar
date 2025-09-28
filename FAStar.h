#pragma once

//import FxsBase;
//import CvGameCoreDLL;

#include <CvDLLFAStarIFaceBase.h>

#include <CommonStuff/vec.h>

#include <vector>

class FAStar
{
public:
	static inline bool sEnableVerificationFixes = false;

	FAStar() = default;
	explicit FAStar(int iColumns, int iRows, bool bWrapX, bool bWrapY, FAPointFunc DestValidFunc, FAHeuristic HeuristicFunc, FAStarFunc CostFunc, FAStarFunc ValidFunc, FAStarFunc NotifyChildFunc, FAStarFunc NotifyListFunc, void* pData);

	bool generatePath(heck::ivec2 start, heck::ivec2 goal, bool bCardinalOnly, int iInfo, bool bReuse);

	void forceReset();

	int getInfo() const;

	heck::ivec2 getStart() const;
	heck::ivec2 getGoal() const;
	FAStarNode* getGoalNode() const;

	void setData(const void* data);
	const void* getData() const;

	bool startGeneratePath(heck::ivec2 start, heck::ivec2 goal, bool bCardinalOnly, int iInfo, bool bReuse);
	bool isReset(heck::ivec2 start, bool bCardinalOnly, int iInfo, bool bReuse) const;
	void setPath(FAStarNode*);
	//void enableVerificationFixes();

	FAStarNode& operator()(int x, int y);
	FAStarNode& operator[](heck::ivec2 coord);

private:
	FAPointFunc mDestValidFunc = nullptr;
	FAHeuristic mHeuristicFunc = nullptr;
	FAStarFunc mCostFunc = nullptr;
	FAStarFunc mValidFunc = nullptr;
	FAStarFunc mNotifyChildFunc = nullptr; // data is ASNC_*
	FAStarFunc mNotifyListFunc = nullptr; // data is ASNL_*
	const void* mUserData = nullptr;
	heck::ivec2 mDim{};
	heck::ivec2 mStart{};
	heck::ivec2 mGoal{};
	int mInfo = 0;
	bool mIsWrappedX = false;
	bool mIsWrappedY = false;
	bool mIsCardinalOnly = false;
	bool mForceReset = false;
	FAStarNode* mOpenListHead = nullptr;
	FAStarNode* mClosedListHead = nullptr;
	FAStarNode* mGoalNode = 0;
	FAStarNode* mUnk48 = nullptr;
	std::vector<FAStarNode> mNodes;

	int mExploredNodeCount = 0;
	//bool mEnableVerificationFixes = false;

	FAStarNode* moveOpenToClosed();
	void visitAdj(FAStarNode* param_2);
	void updateWithStepTo(FAStarNode* param_1, FAStarNode* param_3);
	void updatePositionInOpenList(FAStarNode* param_1);
	void updateClosedSubTree(FAStarNode* param_2);
	void addOpen(FAStarNode* param_2);

	void doVerificationFixes();
};
