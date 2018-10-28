//
// Created by chen on 18-7-11.
//

#ifndef MAPPER_GRID_HPP
#define MAPPER_GRID_HPP

#include "global.hpp"

#include "PointsVector.hpp"

class Grid
{
public:
    PointsVector<PointXYZI> m_stPtsVector;

    PointXYZI m_stHighestPt;

    PointXYZI m_stLowestPt;

    Grid()
    {

    }

    virtual ~Grid()
    {

    }

    /**@brief update the grid by point;
     *
     * update the grid by point;
     * @param pt [IN]: the point used to update the grid;
     * @return void;
     */
    void UpdateByPt(const PointXYZI &pt)
    {
        if (m_stPtsVector.size() == 0)
        {
            m_stHighestPt = pt;
            m_stLowestPt = pt;
        }
        else
        {
            if (PointXYZI::CompareByZ(m_stHighestPt, pt))
            {
                m_stHighestPt = pt;
            }

            if (PointXYZI::CompareByZ(pt, m_stLowestPt))
            {
                m_stLowestPt = pt;
            }
        }
        m_stPtsVector.push_back(pt);
    }

    /**@brief get the number of points falling in the grid;
     *
     * @return the number of points in the grid;
     */
    int GetNumOfPts() const
    {
        return int(m_stPtsVector.size());
    }

    /**@brief get the highest point in the grid;
     *
     * get the highest point in the grid;
     * @param pt [OUT]: highest point in the grid;
     * @return if there is no points in the grid, return false; else, return true;
     */
    bool GetHighestPt(PointXYZI &pt) const
    {
        pt = m_stHighestPt;
        return m_stPtsVector.size() > 0;
    }

    /**@brief get the lowet point in the grid;
     *
     * get the lowest point in the grid;
     * @param pt [OUT]: the lowest point in the grid;
     * @return if there is no points in the grid, return false; else, return true;
     */
    bool GetLowestPt(PointXYZI &pt) const
    {
        pt = m_stLowestPt;
        return m_stPtsVector.size() > 0;
    }

    /**@brief get lowest z in the grid;
     *
     * get lowest z in the grid;
     * @param z [OUT]: the lowest z;
     * @return if there is no points in the grid, return false; else, return true;
     */
    bool GetLowestZ(double &z) const
    {
        z = m_stLowestPt.z();
        return m_stPtsVector.size() > 0;
    }

    /**@brief get highest z in the grid;
     *
     * get highest z in the grid;
     * @param z [OUT]: the highest z;
     * @return if there is no points in the grid, return false; else, return true;
     */
    bool GetHighestZ(double &z) const
    {
        z = m_stHighestPt.z();
        return m_stPtsVector.size() > 0;
    }

    /**@brief get delta z of the points in the grid;
     *
     * get delta z of the points in the grid;
     * @param deltaz [OUT]: the delta z;
     * @return if there is no points in the grid, return false; else, return true;
     */
    bool GetDeltaZ(double &deltaz) const
    {
        deltaz = m_stHighestPt.z() - m_stLowestPt.z();
        return m_stPtsVector.size() > 0;
    }

    /**@brief get mean of the points falling in the grid;
     *
     * @return
     */
    bool GetMeanPt(PointXYZI &pt) const
    {
        return m_stPtsVector.GetMeanPoint(pt);
    }

    /**@brief get all the points in the grid;
     *
     * get all the points in the grid;
     * @param pts [OUT]: the points in the grid will be appended to this vector;
     * @return if there is no points in the grid, return false; else, return true;
     * @note
     */
    bool GetAllPts(std::vector<PointXYZI> &pts) const
    {
        return m_stPtsVector.GetAllPts(pts);
    }

    /**@brief get all the points above threshold in the grid;
     *
     * get all the points above threshold in the grid;
     * @param threshold [IN]: the threshold;
     * @param pts [OUT]: the points int the grid that above the threshold will be appended to this vector;
     * @return if there is no points above the threshold in the grid, return false; else, return true;
     * @note
     */
    bool GetPtsAbove(const double threshold, std::vector<PointXYZI> &pts) const
    {
        size_t size = pts.size();
        for (int p = 0; p < m_stPtsVector.size(); ++p)
        {
            if (m_stPtsVector[p].z()>= threshold)
            {
                pts.push_back(m_stPtsVector[p]);
            }
        }
        return pts.size() - size > 0;
    }

    /**@brief get all the points below threshold in the grid;
     *
     * get all the points below threshold in the grid;
     * @param threshold [IN]: the threshold;
     * @param pts [OUT]: the points in the grid that below the threshold will be appended to this vector;
     * @return if there is no points below the threshold in the grid, return false; else, return true;
     */
    bool GetPtsBelow(const double threshold, std::vector<PointXYZI> &pts) const
    {
        size_t size = pts.size();
        for (int p = 0; p < m_stPtsVector.size(); ++p)
        {
            if (m_stPtsVector[p].z() <= threshold)
            {
                pts.push_back(m_stPtsVector[p]);
            }
        }
        return pts.size() - size > 0;
    }

    /**@brief get the points between two elevation value;
     *
     * get the points between two elevation value;
     * @param low [IN]: the low elevation value;
     * @param high [IN]: the high elevation value;
     * @param pts [OUT]: the vector of the points in the range;
     * @return if there is no points in the elevation range, return false; else, return true;
     */
    bool GetPtsBetween(const double low, const double high, std::vector<PointXYZI> &pts) const
    {
        size_t size = pts.size();
        for (int p = 0; p < m_stPtsVector.size(); ++p)
        {
            if (m_stPtsVector[p].z() >= low && m_stPtsVector[p].z() <= high)
            {
                pts.push_back(m_stPtsVector[p]);
            }
        }
        return pts.size() - size > 0;
    }

    /**@brief remove all the points in the grid;
     *
     */
    void RemoveAllPts()
    {
        m_stPtsVector.clear();
        m_stHighestPt.Reset2Zero();
        m_stLowestPt.Reset2Zero();
    }

    /**@brief remove all the points above threshold;
     *
     * remove the points above threshold;
     * @param threshold [IN]: the threshold;
     * @param removedPts [IN]: if not NULL, the removed points will be pushed into the vector;
     */
    void RemovePtsAbove(const double threshold, std::vector<PointXYZI> *removedPts = NULL)
    {
        if (removedPts == NULL)
        {
            for (std::vector<PointXYZI>::iterator it = m_stPtsVector.begin(); it != m_stPtsVector.end();)
            {
                if ((*it).z() >= threshold)
                {
                    it = m_stPtsVector.erase(it);
                }
                else
                {
                    it++;
                }
            }
        }
        else
        {
            for (std::vector<PointXYZI>::iterator it = m_stPtsVector.begin(); it != m_stPtsVector.end();)
            {
                if ((*it).z() >= threshold)
                {
                    removedPts->push_back(*it);
                    it = m_stPtsVector.erase(it);
                }
                else
                {
                    it++;
                }
            }
        }

    }

    /**@brief remove the points below threshold;
     *
     * remove the points below the threshold;
     * @param threshold [IN]: the threshold;
     * @param removedPts [OUT]: if not null, the removed points will be pushed into this vector;
     */
    void RemovePtsBelow(const double threshold, std::vector<PointXYZI> *removedPts = NULL)
    {
        if (removedPts == NULL)
        {
            for (std::vector<PointXYZI>::iterator it = m_stPtsVector.begin(); it != m_stPtsVector.end();)
            {
                if ((*it).z()<= threshold)
                {
                    it = m_stPtsVector.erase(it);
                }
                else
                {
                    it++;
                }
            }
        }
        else
        {
            for (std::vector<PointXYZI>::iterator it = m_stPtsVector.begin(); it != m_stPtsVector.end();)
            {
                if ((*it).z()<= threshold)
                {
                    removedPts->push_back(*it);
                    it = m_stPtsVector.erase(it);
                }
                else
                {
                    it++;
                }
            }
        }

    }

    /**@brief check whether the grid has points falling into it;
     *
     * @return  if there is points in the grid, return true; else, return false;
     */
    bool HasPts() const
    {
        return !m_stPtsVector.empty();
    }


    /**@brief
     *
     * @param pts
     * @param mean
     * @param variance
     * @return
     */
    bool CalcIntensityStatistics(float &mean, float &std_var)
    {
        if (m_stPtsVector.empty())
        {
            return false;
        }

        float sum = 0.0;
        float squared_sum = 0.0;
        for (const auto& pt: m_stPtsVector)
        {
            sum += pt.i();
            squared_sum += pt.i()*pt.i();
        }

        mean = sum/m_stPtsVector.size();
        std_var = sqrt(squared_sum/m_stPtsVector.size() - mean*mean);
        return true;
    }

    bool GetMinMaxIntensity(float& min_intensity, float& max_intensity)
    {
        min_intensity = 65535;
        max_intensity = 0;
        for (const auto& pt:m_stPtsVector)
        {
            min_intensity = min(pt.i(), min_intensity);
            max_intensity = max(pt.i(), max_intensity);
        }
        return this->HasPts();
    }

    bool GetMaxIntensityPt(PointXYZI& max_pt)
    {
        float max_intensity = 0;
        for(const auto& pt:m_stPtsVector)
        {
            if (pt.i()>max_intensity)
            {
                max_pt = pt;
                max_intensity = max_pt.i();
            }
        }
        return this->HasPts();
    }
};


#endif //MAPPER_GRID_HPP
