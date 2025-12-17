/**
 * Swizzled NavbarContent component to add authentication buttons.
 *
 * This wraps the default Docusaurus navbar content and adds NavbarAuth component.
 */
import React from 'react';
import Content from '@theme-original/Navbar/Content';
import { NavbarAuth } from '../../../components/Auth';

export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <NavbarAuth />
    </>
  );
}
