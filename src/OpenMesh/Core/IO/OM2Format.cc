#include "OM2Format.hh"

namespace OpenMesh {
namespace IO {
namespace OM2Format {
namespace TypeInfo {
enum ElementType { Invalid, Vertex, Halfedge, Edge, Face, Mesh };

struct PropertyDeclaration {
    std::string propertyName;
    std::string typeName;
    uint32_t element = ElementType::Invalid;

    size_t store(std::ostream &os) {
        return OpenMesh::IO::store(os, propertyName) +
               OpenMesh::IO::store(os, typeName) +
               OpenMesh::IO::store(os, element);
    }

    size_t restore(std::istream &is) {
        return OpenMesh::IO::restore(is, propertyName) +
               OpenMesh::IO::restore(is, typeName) +
               OpenMesh::IO::restore(is, element);
    }
};

struct PreambleHeader {
    uint8_t magic[4] = {'O', 'M', '2', '\0'};
    uint32_t version = 1;
    int32_t options = 0;
    uint32_t num_vertex_properties = 0;
    uint32_t num_halfedge_properties = 0;
    uint32_t num_edge_properties = 0;
    uint32_t num_face_properties = 0;
    uint32_t num_mesh_properties = 0;
};

bool checkMagic(const char *first4Bytes) {
    return first4Bytes[0] == 'O' && first4Bytes[1] == 'M' &&
           first4Bytes[2] == '2' && first4Bytes[3] == '\0';
}

void writePreamble(std::ostream &os, const OpenMesh::BaseKernel &mesh,
                   OpenMesh::IO::Options opts) {
    auto savable = [&](OpenMesh::BaseProperty *prop) {
        return (prop && prop->persistent() && !prop->name().empty() &&
                prop->name()[1] != ':');
    };

    PreambleHeader header;
    header.options = int(opts);
    header.num_vertex_properties =
        std::count_if(mesh.vprops_begin(), mesh.vprops_end(), savable);
    header.num_halfedge_properties =
        std::count_if(mesh.hprops_begin(), mesh.hprops_end(), savable);
    header.num_edge_properties =
        std::count_if(mesh.eprops_begin(), mesh.eprops_end(), savable);
    header.num_face_properties =
        std::count_if(mesh.fprops_begin(), mesh.fprops_end(), savable);
    header.num_mesh_properties =
        std::count_if(mesh.mprops_begin(), mesh.mprops_end(), savable);
    os.write(reinterpret_cast<char *>(&header), sizeof(header));

    auto const writeChunk = [&](OpenMesh::BaseProperty const &prop,
                                ElementType type) {
        auto propType = get(prop);
        if (!propType) {
            std::cerr << "Could not store property " << prop.name()
                      << " due to unregistered type." << std::endl;
            return;
        }

        PropertyDeclaration decl;
        decl.element = type;
        decl.propertyName = prop.name();
        decl.typeName = propType->name;
        decl.store(os);
    };

    for (auto it = mesh.vprops_begin(); it != mesh.vprops_end(); it++) {
        if (!savable(*it)) continue;
        writeChunk(**it, ElementType::Vertex);
    }
    for (auto it = mesh.hprops_begin(); it != mesh.hprops_end(); it++) {
        if (!savable(*it)) continue;
        writeChunk(**it, ElementType::Halfedge);
    }
    for (auto it = mesh.eprops_begin(); it != mesh.eprops_end(); it++) {
        if (!savable(*it)) continue;
        writeChunk(**it, ElementType::Edge);
    }
    for (auto it = mesh.fprops_begin(); it != mesh.fprops_end(); it++) {
        if (!savable(*it)) continue;
        writeChunk(**it, ElementType::Face);
    }
    for (auto it = mesh.mprops_begin(); it != mesh.mprops_end(); it++) {
        if (!savable(*it)) continue;
        writeChunk(**it, ElementType::Mesh);
    }
}

ExtendedOptions readPreamble(std::istream &is,
                                   OpenMesh::BaseKernel &mesh) {
    PreambleHeader header;
    is.read(reinterpret_cast<char *>(&header), sizeof(header));
    if (header.magic[0] != 'O' || header.magic[1] != 'M' ||
        header.magic[2] != '2' || header.magic[3] != '\0') {
        std::cerr << "File is missing magic number!" << std::endl;
        return {0};
    }

    const auto props_total =
        header.num_vertex_properties + header.num_halfedge_properties +
        header.num_edge_properties + header.num_face_properties +
        header.num_mesh_properties;

    for (size_t i = 0; i < props_total; i++) {
        PropertyDeclaration decl;
        if (!decl.restore(is)) {
            std::cerr << "Could not read property declaration!" << std::endl;
            return {0};
        }
        auto type = get(decl.typeName);

        switch (decl.element) {
        case ElementType::Vertex:
            type->addToVertex(decl.propertyName, mesh);
            break;
        case ElementType::Halfedge:
            type->addToHalfedge(decl.propertyName, mesh);
            break;
        case ElementType::Edge:
            type->addToEdge(decl.propertyName, mesh);
            break;
        case ElementType::Face:
            type->addToFace(decl.propertyName, mesh);
            break;
        case ElementType::Mesh:
            type->addToMesh(decl.propertyName, mesh);
            break;
        default:
            std::cerr << "Invalid ElementType!" << decl.element << std::endl;
            continue;
        }
    }
    return {header.options};
}

std::vector<std::unique_ptr<BasePropertyType>> sTypes;

} // namespace TypeInfo
BasePropertyType::~BasePropertyType() {}
} // namespace OM2Format
} // namespace IO
} // namespace OpenMesh
