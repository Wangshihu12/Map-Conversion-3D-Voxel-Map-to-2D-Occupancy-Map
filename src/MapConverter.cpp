
#include "MapConverter.hh"

// 这是MapConverter类的构造函数,用于初始化地图转换器
// 参数说明:
// resolution - 地图分辨率
// slopeEstimationSize - 用于坡度估计的窗口大小
// minimumZ - 最小高度阈值
// minimumOccupancy - 最小占用阈值
MapConverter::MapConverter(double resolution, int slopeEstimationSize,
                           double minimumZ, int minimumOccupancy)
{
  // 设置类的成员变量
  MapConverter::resolution = resolution;                   // 设置地图分辨率
  MapConverter::slopeEstimationSize = slopeEstimationSize; // 设置坡度估计窗口大小
  MapConverter::minOcc = minimumOccupancy;                 // 设置最小占用阈值
  MapConverter::minZ = minimumZ;                           // 设置最小高度阈值

  // 创建一个新的2D地图对象并赋值给类成员map
  Map2D m(resolution);
  MapConverter::map = m;
}

MapConverter::~MapConverter() {}

// 这个函数用于更新地图,将体素地图转换为2D地图
// 参数说明:
// vMap - 输入的体素地图
// minMax - 地图的边界值(minX,maxX,minY,maxY,minZ,maxZ)
void MapConverter::updateMap(vector<voxel> vMap, vector<double> minMax)
{
  // 如果体素地图为空则直接返回
  if (vMap.size() == 0)
    return;
  // 检查边界值是否有无穷大,如果有则返回
  for (double mM : minMax)
    if (isinf(mM))
      return; // 如果目标分辨率的体素太少,无法获得地图大小

  // 计算网格中的单元格数量
  int xSize = (minMax[1] - minMax[0]) / MapConverter::map.getResulution();
  int ySize = (minMax[3] - minMax[2]) / MapConverter::map.getResulution();

  // 创建局部高度范围地图
  HeightRangeMap hMap(xSize, ySize);

  // 遍历所有体素,将其添加到高度范围地图中
  for (voxel v : vMap)
  {
    // 由于体素可能大于地图单元格,这个循环遍历体素占据的所有单元格
    int sizeIndex = v.halfSize * 2 / MapConverter::map.getResulution();
    for (int x = 0; x < sizeIndex; x++)
    {
      for (int y = 0; y < sizeIndex; y++)
      {
        // 计算体素在地图中的位置
        int posX = (v.position.x - v.halfSize + resolution * x - minMax[0]) /
                   resolution;
        int posY = (v.position.y - v.halfSize + resolution * y - minMax[2]) /
                   resolution;
        // 如果单元格在局部地图网格外则跳过
        if (posX < 0 || posX >= xSize)
          continue;
        if (posY < 0 || posY >= ySize)
          continue;
        // 高度范围略微扩大以便更好地检测重叠
        heightRange hr = {v.position.z + v.halfSize + resolution * 0.001,
                          v.position.z - v.halfSize - resolution * 0.001};
        hMap.addRange(posX, posY, hr, v.occupied,
                      v.position.x - v.halfSize + resolution * x,
                      v.position.y - v.halfSize + resolution * y);
      }
    }
  }

  // 移除小于机器人安全边距的自由空间
  for (int x = 1; x < xSize - 1; x++)
  {
    for (int y = 1; y < ySize - 1; y++)
    {
      hMap.removeFreeRanges(x, y, minZ);
      int mapValue = 0;
      // 如果没有自由空间,则标记为未知区域(-1)
      if (hMap.free[x][y].size() == 0)
      {
        hMap.posMap[x][y].x = x * resolution + minMax[0];
        hMap.posMap[x][y].y = y * resolution + minMax[2];
        mapValue = -1;
      }
      // 如果当前位置和地图中的值都是未知,则跳过
      if (mapValue == -1 &&
          map.get(hMap.posMap[x][y].x, hMap.posMap[x][y].y) == -1)
        continue;

      // 设置自由空间和高度地图
      map.set(hMap.posMap[x][y].x, hMap.posMap[x][y].y, mapValue);
      if (mapValue == -1)
        continue;
      // 设置底部和顶部高度
      map.setHeight(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                    hMap.free[x][y].back().bottom);
      map.setHeightTop(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                       hMap.free[x][y][0].top);
    }
  }

  // 检查是否有相邻的占据单元格
  for (int x = 2; x < xSize - 2; x++)
  {
    for (int y = 2; y < ySize - 2; y++)
    {
      // 寻找具有未知邻居的自由空间
      if (hMap.free[x][y].size() == 0)
        continue;

      // 检查所有方向的邻居
      for (auto d : DIRECTIONS)
      {
        if (hMap.free[x + d.x][y + d.y].size() != 0)
          continue;
        // 设置占据状态
        map.set(hMap.posMap[x + d.x][y + d.y].x, hMap.posMap[x][y].y,
                hMap.getOccupation(x + d.x, y + d.y, -1, minOcc));
      }
    }
  }

  // 更新坡度地图
  map.updateSlope(slopeEstimationSize, minMax[0], minMax[1], minMax[2],
                  minMax[3]);
}

// 这个函数用于计算一组体素的边界框(bounding box)
// 输入参数是一个体素列表,返回一个包含6个值的向量,表示边界框的最小和最大坐标
vector<double> MapConverter::minMaxVoxel(vector<voxel> list)
{
  // 创建一个长度为6的向量存储结果
  // minMax[0,1] 存储x轴最小最大值
  // minMax[2,3] 存储y轴最小最大值
  // minMax[4,5] 存储z轴最小最大值
  vector<double> minMax(6);

  // 初始化最小值为正无穷,最大值为负无穷
  minMax[0] = INFINITY;  // x min
  minMax[1] = -INFINITY; // x max
  minMax[2] = INFINITY;  // y min
  minMax[3] = -INFINITY; // y max
  minMax[4] = INFINITY;  // z min
  minMax[5] = -INFINITY; // z max

  // 遍历所有体素
  for (voxel v : list)
  {
    // 为了准确估计新区域大小,只使用八叉树中最小的体素
    // 跳过那些大于分辨率0.9倍的体素
    if (v.halfSize > resolution * .9)
      continue;

    // 更新每个轴的最小最大值
    minMax[0] = min(minMax[0], v.position.x); // 更新x最小值
    minMax[1] = max(minMax[1], v.position.x); // 更新x最大值
    minMax[2] = min(minMax[2], v.position.y); // 更新y最小值
    minMax[3] = max(minMax[3], v.position.y); // 更新y最大值
    minMax[4] = min(minMax[4], v.position.z); // 更新z最小值
    minMax[5] = max(minMax[5], v.position.z); // 更新z最大值
  }
  return minMax;
}
