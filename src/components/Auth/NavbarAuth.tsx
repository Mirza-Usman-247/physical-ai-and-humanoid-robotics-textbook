/**
 * NavbarAuth component for authentication buttons in Docusaurus navbar.
 *
 * Features:
 * - Shows "Sign In" and "Sign Up" buttons when not authenticated
 * - Shows user email and "Sign Out" button when authenticated
 * - Opens AuthModal for signin/signup
 * - Integrates with Docusaurus navbar styling
 */
import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import { AuthModal, type AuthModalMode } from './AuthModal';
import './NavbarAuth.css';

export function NavbarAuth() {
  const { user, isAuthenticated, logout, isLoading } = useAuth();
  const [modalOpen, setModalOpen] = useState(false);
  const [modalMode, setModalMode] = useState<AuthModalMode>(null);

  const handleOpenModal = (mode: 'signin' | 'signup') => {
    setModalMode(mode);
    setModalOpen(true);
  };

  const handleCloseModal = () => {
    setModalOpen(false);
    setModalMode(null);
  };

  const handleLogout = async () => {
    try {
      await logout();
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  if (isLoading) {
    return (
      <div className="navbar-auth">
        <span className="navbar-auth-loading">Loading...</span>
      </div>
    );
  }

  if (isAuthenticated && user) {
    return (
      <div className="navbar-auth navbar-auth-authenticated">
        <span className="navbar-auth-user" title={`Logged in as ${user.email}`}>
          👤 {user.email}
        </span>
        <button
          onClick={handleLogout}
          className="navbar-auth-button navbar-auth-button--signout"
        >
          Sign Out
        </button>
      </div>
    );
  }

  return (
    <>
      <div className="navbar-auth navbar-auth-guest">
        <button
          onClick={() => handleOpenModal('signin')}
          className="navbar-auth-button navbar-auth-button--signin"
        >
          Sign In
        </button>
        <button
          onClick={() => handleOpenModal('signup')}
          className="navbar-auth-button navbar-auth-button--signup"
        >
          Sign Up
        </button>
      </div>

      <AuthModal
        isOpen={modalOpen}
        mode={modalMode}
        onClose={handleCloseModal}
        onModeChange={setModalMode}
      />
    </>
  );
}
