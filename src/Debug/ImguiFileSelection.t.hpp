#include "ImguiFileSelection.hpp"

bool fileGetter(void *data, int index, const char** output) {
    std::vector<string>* vec = (std::vector<string>*)data;
    string &s = vec->at(index);
    *output = s.c_str();
    return true;
}

template< class VecType, class StatType, class PointType >
void ImguiFileSelection<VecType, StatType, PointType>::draw() {
    ImGui::ListBox( "files", &m_currentIndex, fileGetter, &m_fileList, m_fileList.size() );
    if( ImGui::Button( "load file" ) ) loadFile();
}

template< class VecType, class StatType, class PointType >
void ImguiFileSelection<VecType, StatType, PointType>::loadFile() {
    m_pointSet->readOpenMesh( std::string( m_fileList[ m_currentIndex ] ) );
    pointSetToPolyscope<VecType, PointType>("point cloud", m_pointSet);
    *m_inputOctree =  InputOctree<VecType, StatType, PointType >( m_pointSet );
    m_inputOctree->fit( 1, 0 );
}

template< class VecType, class StatType, class PointType >
void ImguiFileSelection<VecType, StatType, PointType>::init() {
    for (const auto & entry : fs::directory_iterator(m_pathToDirectory)) {
        std::string s = entry.path().string();
        m_fileList.push_back( s );
    }
}
