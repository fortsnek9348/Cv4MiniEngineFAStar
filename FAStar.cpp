#include "FAStar.h"

//import CommonStd;

#include <CvGameCoreDLL/FAStarNode.h>

#include <array>
#include <assert.h>

FAStar::FAStar(int iColumns, int iRows, bool bWrapX, bool bWrapY, FAPointFunc DestValidFunc, FAHeuristic HeuristicFunc, FAStarFunc CostFunc, FAStarFunc ValidFunc, FAStarFunc NotifyChildFunc, FAStarFunc NotifyListFunc, void* pData)
	: mDestValidFunc(DestValidFunc)
	, mHeuristicFunc(HeuristicFunc)
	, mCostFunc(CostFunc)
	, mValidFunc(ValidFunc)
	, mNotifyChildFunc(NotifyChildFunc)
	, mNotifyListFunc(NotifyListFunc)
	, mUserData(pData)
	, mDim{ iColumns, iRows }
	, mStart{ -1, -1 }
	, mGoal{ -1, -1 }
	, mIsWrappedX(bWrapX)
	, mIsWrappedY(bWrapY)
	, mNodes(iColumns* iRows)
{
	for (int y = 0; y < iRows; ++y)
	{
		for (int x = 0; x < iColumns; ++x)
		{
			FAStarNode& node = (*this)(x, y);
			node.m_iX = x;
			node.m_iY = y;
		}
	}
}

FAStarNode& FAStar::operator()(int x, int y)
{
	assert(x >= 0 && y >= 0);
	assert(x < mDim.x && y < mDim.y);
	return mNodes[x + y * mDim.x];
}

FAStarNode& FAStar::operator[](heck::ivec2 coord)
{
	return (*this)(coord.x, coord.y);
}

bool FAStar::startGeneratePath(heck::ivec2 start, heck::ivec2 goal, bool bCardinalOnly, int iInfo, bool bReuse)
{
	FAStarNode* pFVar1;

	if (0 <= start.x && start.x < mDim.x && 0 <= start.y && start.y < mDim.y)
	{
		mGoal = goal;
		if (!bReuse || mForceReset || mStart != start || mIsCardinalOnly != bCardinalOnly || mInfo != iInfo)
		{
			//if (mExploredNodeCount)
			//{
			//	std::clog << "Resetting path finder at " << mExploredNodeCount << " explored nodes." << std::endl;
			//	if ((!bReuse || mForceReset || mIsCardinalOnly != bCardinalOnly || mInfo != iInfo))
			//		std::clog << "...for reasons other than the start node." << std::endl;
			//}
			mExploredNodeCount = 0;

			FAStarNode* node = mOpenListHead;
			while (node)
			{
				pFVar1 = mOpenListHead;
				node = pFVar1->m_pNext;
				pFVar1->clear();
				mOpenListHead = node;
			}
			node = mClosedListHead;
			while (node)
			{
				pFVar1 = mClosedListHead;
				node = pFVar1->m_pNext;
				pFVar1->clear();
				mClosedListHead = node;
			}
			mInfo = iInfo;
			mIsCardinalOnly = bCardinalOnly;
			mStart = start;
			mGoalNode = nullptr;
			mUnk48 = nullptr;
			mForceReset = false;
			FAStarNode& startNode = (*this)[start];
			startNode.m_iKnownCost = 0;
			startNode.m_iHeuristicCost = mHeuristicFunc ? mHeuristicFunc(mStart.x, mStart.y, mGoal.x, mGoal.y) : 0;
			startNode.m_iTotalCost = startNode.m_iKnownCost + startNode.m_iHeuristicCost;
			mOpenListHead = &startNode;
			if (mNotifyListFunc)
				mNotifyListFunc(0, &startNode, ASNL_STARTOPEN, mUserData, this);
			if (mNotifyChildFunc)
				mNotifyChildFunc(0, &startNode, ASNC_INITIALADD, mUserData, this);
		}
		// Don't need to do this here.
		//else if (mEnableVerificationFixes)
		//	doVerificationFixes();
		if (!mDestValidFunc || mDestValidFunc(goal.x, goal.y, mUserData, this))
			return true;
	}
	return false;
}

bool FAStar::isReset(heck::ivec2 start, bool bCardinalOnly, int iInfo, bool bReuse) const
{
	return !bReuse || mForceReset || mStart != start || mIsCardinalOnly != bCardinalOnly || mInfo != iInfo;
}

bool FAStar::generatePath(heck::ivec2 start, heck::ivec2 goal, bool bCardinalOnly, int iInfo, bool bReuse)
{
	FAStarNode* pFVar1;

	if (0 <= start.x && start.x < mDim.x && 0 <= start.y && start.y < mDim.y)
	{
		mGoal = goal;
		if (!bReuse || mForceReset || mStart != start || mIsCardinalOnly != bCardinalOnly || mInfo != iInfo)
		{
			//std::clog << "Resetting path finder at " << mExploredNodeCount << " explored nodes." << std::endl;
			//if ((!bReuse || mForceReset || mIsCardinalOnly != bCardinalOnly || mInfo != iInfo))
			//	std::clog << "...for reasons other than the start node." << std::endl;
			mExploredNodeCount = 0;

			FAStarNode* node = mOpenListHead;
			while (node)
			{
				pFVar1 = mOpenListHead;
				node = pFVar1->m_pNext;
				pFVar1->clear();
				mOpenListHead = node;
			}
			node = mClosedListHead;
			while (node)
			{
				pFVar1 = mClosedListHead;
				node = pFVar1->m_pNext;
				pFVar1->clear();
				mClosedListHead = node;
			}
			mInfo = iInfo;
			mIsCardinalOnly = bCardinalOnly;
			mStart = start;
			mGoalNode = nullptr;
			mUnk48 = nullptr;
			mForceReset = false;
			FAStarNode& startNode = (*this)[start];
			startNode.m_iKnownCost = 0;
			startNode.m_iHeuristicCost = mHeuristicFunc ? mHeuristicFunc(mStart.x, mStart.y, mGoal.x, mGoal.y) : 0;
			startNode.m_iTotalCost = startNode.m_iKnownCost + startNode.m_iHeuristicCost;
			mOpenListHead = &startNode;
			if (mNotifyListFunc)
				mNotifyListFunc(0, &startNode, ASNL_STARTOPEN, mUserData, this);
			if (mNotifyChildFunc)
				mNotifyChildFunc(0, &startNode, ASNC_INITIALADD, mUserData, this);
		}
		else if (sEnableVerificationFixes)
			doVerificationFixes();

		if (!mDestValidFunc || mDestValidFunc(goal.x, goal.y, mUserData, this))
		{
			if (0 <= goal.x && goal.x < mDim.x && 0 <= goal.y && goal.y < mDim.y && (*this)[goal].m_eFAStarListType == FASTARLIST_CLOSED)
			{
				mGoalNode = &(*this)[goal];
				return true;
			}

			for (;;)
			{
				++mExploredNodeCount;
				mGoalNode = moveOpenToClosed();
				if (!mGoalNode)
					break;
				visitAdj(mGoalNode);
				if (heck::ivec2{ mGoalNode->m_iX, mGoalNode->m_iY } == goal)
					return true;
			}
		}
	}
	return false;
}

void FAStar::forceReset()
{
	mForceReset = true;
}

int FAStar::getInfo() const
{
	return mInfo;
}

heck::ivec2 FAStar::getStart() const
{
	return mStart;
}

heck::ivec2 FAStar::getGoal() const
{
	return mGoal;
}

FAStarNode* FAStar::getGoalNode() const
{
	return mGoalNode;
}

void FAStar::setData(const void* data)
{
	mUserData = data;
}
const void* FAStar::getData() const
{
	return mUserData;
}

void FAStar::setPath(FAStarNode* goal)
{
	mGoalNode = goal;
	if (goal)
		mGoal = { goal->m_iX, goal->m_iY };

	if (mGoalNode)
	{
		FAStarNode* current = mGoalNode;
		while (current->m_pParent)
			current = current->m_pParent;
		mStart = { current->m_iX, current->m_iY };

		current = mGoalNode;
		while (current)
		{
			if (current->m_eFAStarListType == NO_FASTARLIST)
			{
				current->m_eFAStarListType = FASTARLIST_CLOSED;
				current->m_pPrev = nullptr;
				current->m_pNext = mClosedListHead;
				if (mClosedListHead)
					mClosedListHead->m_pPrev = current;
				mClosedListHead = current;
			}
			current = current->m_pParent;
		}
	}
}

FAStarNode* FAStar::moveOpenToClosed()
{
	FAStarNode* pFVar1;
	FAStarNode* pFVar2;

	pFVar1 = mOpenListHead;
	if (!pFVar1)
		return nullptr;

	pFVar2 = pFVar1->m_pNext;
	mOpenListHead = pFVar2;
	if (pFVar2)
		pFVar2->m_pPrev = nullptr;

	if (mNotifyListFunc)
		mNotifyListFunc(0, pFVar1, ASNL_DELETEOPEN, mUserData, this);
	pFVar1->m_eFAStarListType = FASTARLIST_CLOSED;
	pFVar1->m_pNext = mClosedListHead;
	if (mClosedListHead)
		mClosedListHead->m_pPrev = pFVar1;
	mClosedListHead = pFVar1;
	if (mNotifyListFunc)
		mNotifyListFunc(0, pFVar1, ASNL_ADDCLOSED, mUserData, this);

	return pFVar1;
}

void FAStar::visitAdj(FAStarNode* param_2)
{
	static constexpr heck::ivec2 kAdj[]{
		{ -1, 0 },
		{ 0, -1 },
		{ 1, 0 },
		{ 0, 1 },
		{ -1, 1 },
		{ -1, -1 },
		{ 1, -1 },
		{ 1, 1 },
	};

	int iVar4;
	int iVar6;

	for (int iVar7 = 0; iVar7 < (mIsCardinalOnly ? 4 : 8); ++iVar7)
	{
		iVar4 = kAdj[iVar7].x + param_2->m_iX;
		if (mIsWrappedX)
		{
			if (iVar4 < 0)
				iVar4 = iVar4 % mDim.x + mDim.x;
			else if (mDim.x <= iVar4)
				iVar4 %= mDim.x;
		}
		iVar6 = kAdj[iVar7].y + param_2->m_iY;
		if (mIsWrappedY)
		{
			if (iVar6 < 0)
				iVar6 = iVar6 % mDim.y + mDim.y;
			else if (mDim.y <= iVar6)
				iVar6 %= mDim.y;
		}
		if (0 <= iVar4 && iVar4 < mDim.x && 0 <= iVar6 && iVar6 < mDim.y)
		{
			FAStarNode& node = (*this)[{ iVar4, iVar6 }];
			if (!mValidFunc || mValidFunc(param_2, &node, 0, mUserData, this))
				updateWithStepTo(param_2, &node);
		}
	}
}

void FAStar::updateWithStepTo(FAStarNode* param_1, FAStarNode* param_3)
{
	const int iVar1 = param_1->m_iKnownCost + (mCostFunc ? mCostFunc(param_1, param_3, 0, mUserData, this) : 1);

	if (param_3->m_eFAStarListType == FASTARLIST_OPEN)
	{
		param_1->m_apChildren[param_1->m_iNumChildren] = param_3;
		param_1->m_iNumChildren = param_1->m_iNumChildren + 1;
		if (iVar1 < param_3->m_iKnownCost)
		{
			param_3->m_pParent = param_1;
			param_3->m_iKnownCost = iVar1;
			param_3->m_iTotalCost = param_3->m_iHeuristicCost + iVar1;
			updatePositionInOpenList(param_3);
			if (mNotifyChildFunc)
				mNotifyChildFunc(param_1, param_3, ASNC_OPENADD_UP, mUserData, this);
		}
	}
	else if (param_3->m_eFAStarListType == FASTARLIST_CLOSED)
	{
		param_1->m_apChildren[param_1->m_iNumChildren] = param_3;
		param_1->m_iNumChildren = param_1->m_iNumChildren + 1;
		if (iVar1 < param_3->m_iKnownCost)
		{
			param_3->m_pParent = param_1;
			param_3->m_iKnownCost = iVar1;
			param_3->m_iTotalCost = param_3->m_iHeuristicCost + iVar1;
			if (mNotifyChildFunc)
				mNotifyChildFunc(param_1, param_3, ASNC_CLOSEDADD_UP, mUserData, this);
			updateClosedSubTree(param_3);
		}
	}
	else
	{
		param_3->m_pParent = param_1;
		param_3->m_iKnownCost = iVar1;
		param_3->m_iHeuristicCost = mHeuristicFunc ? mHeuristicFunc(param_3->m_iX, param_3->m_iY, mGoal.x, mGoal.y) : 0;
		param_3->m_iTotalCost = param_3->m_iKnownCost + param_3->m_iHeuristicCost;
		addOpen(param_3);
		param_1->m_apChildren[param_1->m_iNumChildren] = param_3;
		param_1->m_iNumChildren = param_1->m_iNumChildren + 1;
		if (mNotifyChildFunc)
			mNotifyChildFunc(param_1, param_3, ASNC_NEWADD, mUserData, this);
	}
}

void FAStar::updatePositionInOpenList(FAStarNode* param_1)
{
	FAStarNode* pFVar1;

	pFVar1 = param_1->m_pNext;
	while (pFVar1 && (pFVar1 = param_1->m_pNext, pFVar1->m_iTotalCost < param_1->m_iTotalCost))
	{
		if (pFVar1->m_pNext)
			pFVar1->m_pNext->m_pPrev = param_1;
		if (param_1->m_pPrev)
			param_1->m_pPrev->m_pNext = pFVar1;
		param_1->m_pNext = pFVar1->m_pNext;
		pFVar1->m_pPrev = param_1->m_pPrev;
		pFVar1->m_pNext = param_1;
		param_1->m_pPrev = pFVar1;
		if (mOpenListHead == param_1)
			mOpenListHead = pFVar1;
		pFVar1 = param_1->m_pNext;
	}

	pFVar1 = param_1->m_pPrev;
	while (pFVar1 && (pFVar1 = param_1->m_pPrev, param_1->m_iTotalCost < pFVar1->m_iTotalCost))
	{
		if (pFVar1->m_pPrev)
			pFVar1->m_pPrev->m_pNext = param_1;
		if (param_1->m_pNext)
			param_1->m_pNext->m_pPrev = pFVar1;
		param_1->m_pPrev = pFVar1->m_pPrev;
		pFVar1->m_pNext = param_1->m_pNext;
		pFVar1->m_pPrev = param_1;
		param_1->m_pNext = pFVar1;
		if (mOpenListHead == pFVar1)
			mOpenListHead = param_1;
		pFVar1 = param_1->m_pPrev;
	}
}

void FAStar::updateClosedSubTree(FAStarNode* root)
{
	FAStarNode* child;
	int updatedKnownCost;
	FAStarNode* parent;

	parent = root;
	if (root)
	{
		for (;;)
		{
			for (int i = 0, n = parent->m_iNumChildren; i < n; ++i)
			{
				child = parent->m_apChildren[i];
				updatedKnownCost = parent->m_iKnownCost + (mCostFunc ? mCostFunc(parent, child, 0, mUserData, this) : 1);
				if (updatedKnownCost < child->m_iKnownCost)
				{
					child->m_iKnownCost = updatedKnownCost;
					child->m_iTotalCost = child->m_iHeuristicCost + updatedKnownCost;
					child->m_pParent = parent;
					if (child->m_eFAStarListType == FASTARLIST_OPEN)
						updatePositionInOpenList(child);

					if (mNotifyChildFunc)
						mNotifyChildFunc(parent, child, ASNC_PARENTADD_UP, mUserData, this);

					if (!child->m_bOnStack)
					{
						if (!mUnk48)
							mUnk48 = &(*this)[{ child->m_iX, child->m_iY }];
						else
						{
							child->m_pStack = mUnk48;
							mUnk48 = child;
						}
						child->m_bOnStack = true;
					}
				}
			}
			parent = mUnk48;
			if (!parent)
				break;
			mUnk48 = parent->m_pStack;
			parent->m_pStack = nullptr;
			parent->m_bOnStack = false;
		}
	}
}

void FAStar::addOpen(FAStarNode* param_2)
{
	FAStarNode* pFVar1;
	FAStarNode* pFVar2;
	FAStarNode* pFVar3;
	FAStarNode* pFVar4;

	param_2->m_eFAStarListType = FASTARLIST_OPEN;
	pFVar1 = mOpenListHead;
	if (!pFVar1)
	{
		mOpenListHead = param_2;
		param_2->m_pNext = nullptr;
		mOpenListHead->m_pPrev = nullptr;
		if (mNotifyListFunc)
			mNotifyListFunc(0, param_2, ASNL_STARTOPEN, mUserData, this);
	}
	else
	{
		pFVar2 = pFVar1;
		pFVar4 = nullptr;
		do {
			pFVar3 = pFVar2;
			if (param_2->m_iTotalCost <= pFVar3->m_iTotalCost)
			{
				if (!pFVar4)
				{
					param_2->m_pNext = pFVar1;
					mOpenListHead->m_pPrev = param_2;
					mOpenListHead = param_2;
					if (mNotifyListFunc)
						mNotifyListFunc(param_2->m_pNext, param_2, ASNL_STARTOPEN, mUserData, this);
				}
				else
				{
					pFVar4->m_pNext = param_2;
					param_2->m_pPrev = pFVar4;
					param_2->m_pNext = pFVar3;
					pFVar3->m_pPrev = param_2;
					if (mNotifyListFunc)
						mNotifyListFunc(pFVar4, param_2, ASNL_ADDOPEN, mUserData, this);
				}
				return;
			}
			pFVar2 = pFVar3->m_pNext;
			pFVar4 = pFVar3;
		} while (pFVar3->m_pNext);

		pFVar3->m_pNext = param_2;
		param_2->m_pPrev = pFVar3;

		if (mNotifyListFunc)
			mNotifyListFunc(pFVar4, param_2, ASNL_ADDOPEN, mUserData, this);
	}
}

void FAStar::doVerificationFixes()
{
	// If there's no heuristic, then fcosts didn't change.
	if (!mHeuristicFunc)
		return;

	// Go through the open set and update the F costs.
	FAStarNode* node = mOpenListHead;
	while (node)
	{
		node->m_iHeuristicCost = mHeuristicFunc(node->m_iX, node->m_iY, mGoal.x, mGoal.y);
		const int fcost = node->m_iKnownCost + node->m_iHeuristicCost;
		node->m_iTotalCost = fcost;
		FAStarNode* const next = node->m_pNext;
		// Sort by shifting to the start. We assume the list is mostly sorted already.
		while (node->m_pPrev && fcost < node->m_pPrev->m_iTotalCost)
		{
			FAStarNode* const prev2 = node->m_pPrev->m_pPrev;
			FAStarNode* const prev1 = node->m_pPrev;
			FAStarNode* const next2 = node->m_pNext;
			(prev2 ? prev2->m_pNext : mOpenListHead) = node;
			node->m_pPrev = prev2;
			node->m_pNext = prev1;
			prev1->m_pPrev = node;
			prev1->m_pNext = next2;
			if (next2)
				next2->m_pPrev = prev1;
		}
		node = next;
	}

	//node = mOpenListHead;
	//while (node && node->m_pNext)
	//	if (node->m_iTotalCost
}