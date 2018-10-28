//
// Created by chen on 18-7-20.
//

#ifndef LOCALIZER_ROWGRIDS_HPP
#define LOCALIZER_ROWGRIDS_HPP

#include <global.hpp>
#include "Grid.hpp"

class RowGrids
{
public:
    RowGrids()
    {

    }

    ~RowGrids()
    {
        if (m_pstGrids!=NULL)
        {
            delete[](m_pstGrids);
            m_pstGrids = NULL;
        }
    }

    void SetInputPoints(double bound_y, const PointsVector<PointXYZI>& pts)
    {
        m_nNumOfGrids = int(fabs(bound_y*10)+1);
        m_pstGrids = new Grid[m_nNumOfGrids];

        for (const auto& pt:pts)
        {
            auto idx = int(fabs(pt.y()*10));
            m_pstGrids[idx].UpdateByPt(pt);//.m_stPtsVector.emplace_back(pt);
        }
    }

    bool GetEdgeAndMarkerPoints(PointXYZ& edge, PointsVector<PointXYZI>& markers)
    {
        double last_lowest_z = 0.0;
        double last_highest_z = 0.0;
        int i = 0;

        //find first point
        for (i = 0; i < m_nNumOfGrids; ++i)
        {
            if (m_pstGrids[i].HasPts())
            {
                m_pstGrids[i].GetLowestZ(last_lowest_z);
                m_pstGrids[i].GetHighestZ(last_highest_z);
                break;
            }
        }

        double lowest_z, highest_z;
        PointXYZI temp_pt;
        for (; i<m_nNumOfGrids; ++i)
        {
            if (m_pstGrids[i].HasPts())
            {
                m_pstGrids[i].GetLowestZ(lowest_z);
                m_pstGrids[i].GetHighestZ(highest_z);

                if (highest_z-lowest_z>0.1 || highest_z-last_lowest_z>0.1 || last_highest_z-lowest_z>0.1)
                {
                    m_pstGrids[i].GetHighestPt(temp_pt);
//                    m_pstGrids[i].GetMeanPt(temp_pt);
                    edge.SetXYZ(temp_pt.x(), temp_pt.y(), 0.0);
                    return true;
                }
                else
                {
                    last_lowest_z = lowest_z;
                    last_highest_z = highest_z;
                }

                PointXYZI max_intensity_pt;
                m_pstGrids[i].GetMaxIntensityPt(max_intensity_pt);
                if (max_intensity_pt.i()>FLAGS_marker_intensity)
                {
                    markers.emplace_back(max_intensity_pt);
                }
            }
        }
        return false;
    }

private:
    Grid* m_pstGrids;
    int m_nNumOfGrids;
};


#endif //LOCALIZER_ROWGRIDS_HPP
