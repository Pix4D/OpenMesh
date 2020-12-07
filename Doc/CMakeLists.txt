include (VCIDoxygen)

IF (DOXYGEN_FOUND)

  # Add a documentation install target
  vci_create_doc_target(doc-install)

  if (TARGET doc-install)
    vci_copy_after_build (doc-install "${CMAKE_BINARY_DIR}/Build/${ACG_PROJECT_DATADIR}/Doc/html"   "${CMAKE_INSTALL_PREFIX}/${ACG_PROJECT_DATADIR}/doc/html")
  endif()

ENDIF(DOXYGEN_FOUND)
