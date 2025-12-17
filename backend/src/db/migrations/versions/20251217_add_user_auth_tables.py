"""add_user_auth_tables

Revision ID: f8b3c4d72e91
Revises: 5d887276f91d
Create Date: 2025-12-17 05:00:00.000000

"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import postgresql

# revision identifiers, used by Alembic.
revision = 'f8b3c4d72e91'
down_revision = '5d887276f91d'
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Rename existing users table to avoid conflict
    op.rename_table('users', 'rag_users')

    # Create new authentication users table
    op.create_table('auth_users',
        sa.Column('id', postgresql.UUID(as_uuid=True), nullable=False, server_default=sa.text('gen_random_uuid()')),
        sa.Column('email', sa.String(length=255), nullable=False),
        sa.Column('password_hash', sa.String(length=255), nullable=False),
        sa.Column('is_active', sa.Boolean(), nullable=False, server_default=sa.text('true')),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.PrimaryKeyConstraint('id'),
        sa.UniqueConstraint('email'),
        sa.CheckConstraint("email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\\.[A-Z|a-z]{2,}$'", name='email_format_check')
    )
    op.create_index(op.f('ix_auth_users_email'), 'auth_users', ['email'], unique=True)

    # Create user_profiles table
    op.create_table('user_profiles',
        sa.Column('user_id', postgresql.UUID(as_uuid=True), nullable=False),
        sa.Column('ai_level', sa.Integer(), nullable=False),
        sa.Column('ml_level', sa.Integer(), nullable=False),
        sa.Column('ros_level', sa.Integer(), nullable=False),
        sa.Column('python_level', sa.Integer(), nullable=False),
        sa.Column('linux_level', sa.Integer(), nullable=False),
        sa.Column('has_gpu', sa.Boolean(), nullable=False, server_default=sa.text('false')),
        sa.Column('has_jetson', sa.Boolean(), nullable=False, server_default=sa.text('false')),
        sa.Column('has_robot', sa.Boolean(), nullable=False, server_default=sa.text('false')),
        sa.Column('cloud_only', sa.Boolean(), sa.Computed('(NOT has_gpu AND NOT has_jetson AND NOT has_robot)', persisted=True), nullable=False),
        sa.Column('created_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.Column('updated_at', sa.TIMESTAMP(timezone=True), nullable=False, server_default=sa.text('NOW()')),
        sa.ForeignKeyConstraint(['user_id'], ['auth_users.id'], ondelete='CASCADE'),
        sa.PrimaryKeyConstraint('user_id'),
        sa.CheckConstraint('ai_level >= 1 AND ai_level <= 5', name='ai_level_range_check'),
        sa.CheckConstraint('ml_level >= 1 AND ml_level <= 5', name='ml_level_range_check'),
        sa.CheckConstraint('ros_level >= 1 AND ros_level <= 5', name='ros_level_range_check'),
        sa.CheckConstraint('python_level >= 1 AND python_level <= 5', name='python_level_range_check'),
        sa.CheckConstraint('linux_level >= 1 AND linux_level <= 5', name='linux_level_range_check')
    )
    op.create_index(op.f('ix_user_profiles_user_id'), 'user_profiles', ['user_id'], unique=True)

    # Create trigger to auto-update updated_at timestamp
    op.execute("""
        CREATE OR REPLACE FUNCTION update_updated_at_column()
        RETURNS TRIGGER AS $$
        BEGIN
            NEW.updated_at = NOW();
            RETURN NEW;
        END;
        $$ language 'plpgsql';
    """)

    op.execute("""
        CREATE TRIGGER update_auth_users_updated_at BEFORE UPDATE ON auth_users
        FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
    """)

    op.execute("""
        CREATE TRIGGER update_user_profiles_updated_at BEFORE UPDATE ON user_profiles
        FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
    """)


def downgrade() -> None:
    # Drop triggers
    op.execute('DROP TRIGGER IF EXISTS update_user_profiles_updated_at ON user_profiles;')
    op.execute('DROP TRIGGER IF EXISTS update_auth_users_updated_at ON auth_users;')
    op.execute('DROP FUNCTION IF EXISTS update_updated_at_column();')

    # Drop tables
    op.drop_index(op.f('ix_user_profiles_user_id'), table_name='user_profiles')
    op.drop_table('user_profiles')
    op.drop_index(op.f('ix_auth_users_email'), table_name='auth_users')
    op.drop_table('auth_users')

    # Restore original users table name
    op.rename_table('rag_users', 'users')
